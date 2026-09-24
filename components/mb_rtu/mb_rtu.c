#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "cfg.h"
#include "meas_core.h"
#include "mb_rtu.h"

#define RS485_UART      UART_NUM_1
#define RESP_TIMEOUT_MS 300
#define TURNAROUND_MS   5   /* >= 3.5 char times at 9600 baud */

static const char *TAG = "mbrtu";

static SemaphoreHandle_t s_mtx;   /* NULL = bus not initialized */

/* standard Modbus CRC16 (poly 0xA001, init 0xFFFF, little-endian on wire) */
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

/* Send request (CRC appended here) and read exactly resp_len bytes.
 * Caller holds the mutex. */
static esp_err_t transact(uint8_t *req, size_t req_len,
                          uint8_t *resp, size_t resp_len)
{
    uint16_t crc = crc16(req, req_len);
    req[req_len]     = (uint8_t)(crc & 0xFF);
    req[req_len + 1] = (uint8_t)(crc >> 8);

    uart_flush_input(RS485_UART);
    vTaskDelay(pdMS_TO_TICKS(TURNAROUND_MS)); /* inter-frame silence */
    uart_write_bytes(RS485_UART, req, req_len + 2);
    ESP_ERROR_CHECK(uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(100)));
    /* Auto-direction transceivers echo our own TX back into RX; drop it
     * NOW — the slave must stay silent >=3.5 char times (~4 ms @9600)
     * before replying, so the reply cannot have started yet. Without
     * this, the reply parse starts at our own request bytes -> CRC fail
     * on every read (first symptom seen on real hardware, 15/07). */
    uart_flush_input(RS485_UART);

    int got = uart_read_bytes(RS485_UART, resp, resp_len,
                              pdMS_TO_TICKS(RESP_TIMEOUT_MS));
    if (got < 3) {
        return ESP_ERR_TIMEOUT;
    }
    if (resp[1] & 0x80) { /* exception: [addr][func|80][code][crc] */
        ESP_LOGW(TAG, "slave %u exception %u on func %u",
                 resp[0], got >= 3 ? resp[2] : 0, resp[1] & 0x7F);
        return ESP_FAIL;
    }
    if (got < (int)resp_len) {
        return ESP_ERR_TIMEOUT;
    }
    uint16_t rcrc = (uint16_t)resp[resp_len - 1] << 8 | resp[resp_len - 2];
    if (crc16(resp, resp_len - 2) != rcrc) {
        ESP_LOGW(TAG, "crc error from slave %u", resp[0]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mb_rtu_read_regs(uint8_t slave, uint8_t func, uint16_t reg,
                           uint16_t n, uint16_t *out)
{
    if (s_mtx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (n == 0 || n > 4) {
        return ESP_ERR_INVALID_ARG;
    }
    /* func 1/2 read n BITS (coils / discrete inputs), out[i] = 0/1;
     * func 3/4 read n 16-bit registers. */
    bool bits = (func == 1 || func == 2);
    uint8_t databytes = bits ? (uint8_t)((n + 7) / 8) : (uint8_t)(2 * n);

    uint8_t req[8] = {
        slave, func,
        (uint8_t)(reg >> 8), (uint8_t)reg,
        (uint8_t)(n >> 8), (uint8_t)n,
    };
    /* response: addr func bytecount data[databytes] crc[2] */
    uint8_t resp[3 + 8 + 2];
    size_t resp_len = 3 + databytes + 2;

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = transact(req, 6, resp, resp_len);
    xSemaphoreGive(s_mtx);
    if (err != ESP_OK) {
        return err;
    }
    if (resp[0] != slave || resp[2] != databytes) {
        return ESP_FAIL;
    }
    if (bits) {
        for (uint16_t i = 0; i < n; i++) {
            out[i] = (resp[3 + i / 8] >> (i % 8)) & 0x01;
        }
        return ESP_OK;
    }
    for (uint16_t i = 0; i < n; i++) {
        out[i] = (uint16_t)resp[3 + 2 * i] << 8 | resp[4 + 2 * i];
    }
    return ESP_OK;
}

esp_err_t mb_rtu_write_coils(uint8_t slave, uint16_t start, uint16_t n,
                             const uint8_t *bits)
{
    if (s_mtx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (n == 0 || n > 16) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t nbytes = (uint8_t)((n + 7) / 8);
    uint8_t req[9 + 2 + 2] = {
        slave, 0x0F,
        (uint8_t)(start >> 8), (uint8_t)start,
        (uint8_t)(n >> 8), (uint8_t)n,
        nbytes,
    };
    memcpy(&req[7], bits, nbytes);
    /* response echo: addr func startH startL cntH cntL crc2 = 8 bytes */
    uint8_t resp[8];

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = transact(req, 7 + nbytes, resp, sizeof(resp));
    xSemaphoreGive(s_mtx);
    if (err != ESP_OK) {
        return err;
    }
    return (resp[0] == slave && resp[1] == 0x0F) ? ESP_OK : ESP_FAIL;
}

/* ------------------------- sensor poller (mbrtu channels) -------------- */

static float decode_value(const cfg_channel_t *c, const uint16_t *regs)
{
    /* same decode as mb_tcp: ABCD default, word_swap = CDAB */
    uint32_t raw = c->word_swap ? (((uint32_t)regs[1] << 16) | regs[0])
                                : (((uint32_t)regs[0] << 16) | regs[1]);
    switch (c->dtype) {
    case DT_I16: return (float)(int16_t)regs[0];
    case DT_U32: return (float)raw;
    case DT_I32: return (float)(int32_t)raw;
    case DT_F32: {
        float f;
        memcpy(&f, &raw, sizeof(f));
        return f;
    }
    case DT_U16:
    default:     return (float)regs[0];
    }
}

static void poller_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    int64_t next_due[CFG_MAX_CHANNELS] = { 0 };

    while (1) {
        int64_t now = esp_timer_get_time();
        for (size_t i = 0; i < count && i < CFG_MAX_CHANNELS; i++) {
            const cfg_channel_t *c = &chans[i];
            if (c->bus != BUS_MBRTU || now < next_due[i]) {
                continue;
            }
            bool is_bits = (c->func == 1 || c->func == 2);
            uint16_t n_regs = (!is_bits &&
                               (c->dtype == DT_U32 || c->dtype == DT_I32 ||
                                c->dtype == DT_F32)) ? 2 : 1;
            uint16_t regs[2] = { 0 };
            esp_err_t err = mb_rtu_read_regs(c->unit, c->func, c->reg,
                                             n_regs, regs);
            if (err == ESP_OK) {
                meas_push(c->id, SRC_MBTCP, Q_GOOD,
                          decode_value(c, regs) * c->scale + c->offset);
            } else {
                meas_push(c->id, SRC_MBTCP, Q_COMM_ERR, 0.0f);
            }
            next_due[i] = esp_timer_get_time() + (int64_t)c->period_ms * 1000;
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t mb_rtu_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    bool have_sensors = false;
    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus == BUS_MBRTU) {
            have_sensors = true;
            break;
        }
    }
#if CONFIG_FMS_TOWER_ENABLE && CONFIG_FMS_TOWER_BACKEND_MBRTU
    bool need_bus = true;   /* tower relay module lives on this bus */
#else
    bool need_bus = have_sensors;
#endif
    if (!need_bus) {
        ESP_LOGI(TAG, "rs485 unused (no mbrtu channels, tower not on bus)");
        return ESP_OK;
    }

    const uart_config_t uc = {
        .baud_rate  = CONFIG_FMS_RS485_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART, 512, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &uc));
#if CONFIG_FMS_RS485_EN_GPIO >= 0
    /* EN (DE/RE) cua transceiver lai bang RTS o che do RS485 half-duplex:
     * driver keo CAO khi phat, HA xuong de nhan. Khong lai chan nay thi
     * bo thu bi khoa — node phat duoc (den cam bien chop) nhung khong bao
     * gio nghe thay tra loi (bug tim ra tren board that, 15/07/2026). */
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART,
                                 CONFIG_FMS_RS485_TX_GPIO,
                                 CONFIG_FMS_RS485_RX_GPIO,
                                 CONFIG_FMS_RS485_EN_GPIO,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART, UART_MODE_RS485_HALF_DUPLEX));
#else
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART,
                                 CONFIG_FMS_RS485_TX_GPIO,
                                 CONFIG_FMS_RS485_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif

    s_mtx = xSemaphoreCreateMutex();
    if (s_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "rs485 ready: uart%d tx=%d rx=%d en=%d %d-8N1", RS485_UART,
             CONFIG_FMS_RS485_TX_GPIO, CONFIG_FMS_RS485_RX_GPIO,
             CONFIG_FMS_RS485_EN_GPIO, CONFIG_FMS_RS485_BAUD);

    if (have_sensors) {
        BaseType_t ok = xTaskCreatePinnedToCore(poller_task, "mbrtu_poll",
                                                3072, NULL, 10, NULL, 1);
        return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
