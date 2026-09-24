#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "cfg.h"
#include "meas_core.h"
#include "i2c_sens.h"

#define I2C_TIMEOUT_MS 100
#define MAX_DEVS       4

static const char *TAG = "i2csens";

typedef enum { CHIP_SHT3X, CHIP_AHT20 } chip_t;

typedef struct {
    chip_t  chip;
    uint8_t addr;
    i2c_master_dev_handle_t dev;
    bool    inited;
} sens_dev_t;

static i2c_master_bus_handle_t s_bus;
static sens_dev_t s_devs[MAX_DEVS];
static size_t     s_dev_count;

/* CRC-8 poly 0x31 init 0xFF — used by both SHT3x and AHT20 */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

/* One combined temp+humidity reading in engineering units. */
static bool read_sht3x(sens_dev_t *d, float *temp, float *humid)
{
    /* single-shot, high repeatability, no clock stretching */
    const uint8_t cmd[2] = { 0x24, 0x00 };
    if (i2c_master_transmit(d->dev, cmd, sizeof(cmd), I2C_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(16)); /* max measurement time ~15 ms */

    uint8_t raw[6];
    if (i2c_master_receive(d->dev, raw, sizeof(raw), I2C_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
    if (crc8(raw, 2) != raw[2] || crc8(raw + 3, 2) != raw[5]) {
        ESP_LOGW(TAG, "sht3x@0x%02x crc error", d->addr);
        return false;
    }
    uint16_t t = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t h = ((uint16_t)raw[3] << 8) | raw[4];
    *temp  = -45.0f + 175.0f * (float)t / 65535.0f;
    *humid = 100.0f * (float)h / 65535.0f;
    return true;
}

static bool read_aht20(sens_dev_t *d, float *temp, float *humid)
{
    if (!d->inited) {
        const uint8_t init_cmd[3] = { 0xBE, 0x08, 0x00 };
        (void)i2c_master_transmit(d->dev, init_cmd, sizeof(init_cmd),
                                  I2C_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(10));
        d->inited = true;
    }

    const uint8_t trigger[3] = { 0xAC, 0x33, 0x00 };
    if (i2c_master_transmit(d->dev, trigger, sizeof(trigger),
                            I2C_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(85)); /* measurement takes ~80 ms */

    uint8_t raw[7];
    if (i2c_master_receive(d->dev, raw, sizeof(raw), I2C_TIMEOUT_MS) != ESP_OK) {
        return false;
    }
    if (raw[0] & 0x80) { /* busy bit still set */
        return false;
    }
    if (crc8(raw, 6) != raw[6]) {
        ESP_LOGW(TAG, "aht20@0x%02x crc error", d->addr);
        return false;
    }
    uint32_t h = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | (raw[3] >> 4);
    uint32_t t = (((uint32_t)raw[3] & 0x0F) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
    *humid = (float)h / 1048576.0f * 100.0f;
    *temp  = (float)t / 1048576.0f * 200.0f - 50.0f;
    return true;
}

static sens_dev_t *dev_for(const cfg_channel_t *c)
{
    chip_t chip = (strcmp(c->chip, "aht20") == 0) ? CHIP_AHT20 : CHIP_SHT3X;
    uint8_t addr = c->addr;
    if (addr == 0) {
        addr = (chip == CHIP_AHT20) ? 0x38 : 0x44;
    }

    for (size_t i = 0; i < s_dev_count; i++) {
        if (s_devs[i].chip == chip && s_devs[i].addr == addr) {
            return &s_devs[i];
        }
    }
    if (s_dev_count == MAX_DEVS) {
        return NULL;
    }

    sens_dev_t *d = &s_devs[s_dev_count];
    d->chip = chip;
    d->addr = addr;
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
    };
    if (i2c_master_bus_add_device(s_bus, &dc, &d->dev) != ESP_OK) {
        ESP_LOGE(TAG, "add device 0x%02x failed", addr);
        return NULL;
    }
    s_dev_count++;
    ESP_LOGI(TAG, "%s @0x%02x registered",
             chip == CHIP_AHT20 ? "aht20" : "sht3x", addr);
    return d;
}

static void poll_channel(const cfg_channel_t *c)
{
    sens_dev_t *d = dev_for(c);
    if (d == NULL) {
        return;
    }

    float temp, humid;
    bool ok = (d->chip == CHIP_AHT20) ? read_aht20(d, &temp, &humid)
                                      : read_sht3x(d, &temp, &humid);
    if (!ok) {
        meas_push(c->id, SRC_I2C, Q_COMM_ERR, 0.0f);
        return;
    }
    float v = (strcmp(c->meas, "humid") == 0) ? humid : temp;
    meas_push(c->id, SRC_I2C, Q_GOOD, v * c->scale + c->offset);
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
            if (chans[i].bus != BUS_I2C) {
                continue;
            }
            if (now >= next_due[i]) {
                poll_channel(&chans[i]);
                next_due[i] = esp_timer_get_time() +
                              (int64_t)chans[i].period_ms * 1000;
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t i2c_sens_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    bool any = false;
    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus == BUS_I2C) {
            any = true;
            break;
        }
    }
    if (!any) {
        ESP_LOGI(TAG, "no i2c channels configured, poller not started");
        return ESP_OK;
    }

    i2c_master_bus_config_t bc = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = CONFIG_FMS_I2C_SDA_GPIO,
        .scl_io_num        = CONFIG_FMS_I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* most modules also have their own */
    };
    esp_err_t err = i2c_new_master_bus(&bc, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "i2c bus ready sda=%d scl=%d",
             CONFIG_FMS_I2C_SDA_GPIO, CONFIG_FMS_I2C_SCL_GPIO);

    BaseType_t ok = xTaskCreatePinnedToCore(poller_task, "i2c_poll", 4096,
                                            NULL, 8, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
