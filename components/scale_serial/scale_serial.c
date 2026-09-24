#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "cfg.h"
#include "meas_core.h"
#include "scale_parse.h"
#include "scale_serial.h"
#include "uplink.h"

#define SCALE_LINE_MAX  64
#define RX_BUF_SIZE     512
#define CHANGE_EPSILON  0.01f
#define STABLE_FORCE_MS 5000
/* 21/09: 1000 -> 200. Day chinh la "it nhat 1 giay moi thay gia tri moi"
 * ma nguoi dung bao: can chi phat toi da 1 mau/giay khi vat con dao dong.
 * Do duoc: vong Odoo->node->Odoo chi 0,49 s, nen 1 s nay la tran do chinh
 * firmware tu dat, khong phai duong truyen. */
#define UNSTABLE_MIN_MS 200

static const char *TAG = "scale";

/* HAI cong serial doc lap. Cong 1 = can KENDY (header TXD43/RXD44 qua
 * MAX3232). Cong 2 = UART0 (trong tu khi console doi sang USB-JTAG) —
 * can thu hai, cau HC-05 (Bluetooth Classic -> UART), dau doc ma vach...
 * Kenh Odoo chon cong bang truong "Serial Port" (JSON key "uart" 1|2). */
typedef struct {
    uart_port_t          uart_num;
    int                  tx_gpio;
    int                  rx_gpio;
    const cfg_channel_t *ch;              /* kenh serial gan vao cong nay */
    /* trang thai report rieng tung cong */
    float                last_value;
    int64_t              last_stable_us;
    int64_t              last_unstable_us;
} sport_t;

static sport_t s_ports[2] = {
    { .uart_num = (uart_port_t)CONFIG_FMS_SCALE_UART_NUM,
      .tx_gpio  = CONFIG_FMS_SCALE_UART_TX_GPIO,
      .rx_gpio  = CONFIG_FMS_SCALE_UART_RX_GPIO },
    { .uart_num = (uart_port_t)CONFIG_FMS_SERIAL2_UART_NUM,
      .tx_gpio  = CONFIG_FMS_SERIAL2_TX_GPIO,
      .rx_gpio  = CONFIG_FMS_SERIAL2_RX_GPIO },
};

static void report(sport_t *p, const scale_reading_t *r)
{
    int64_t now = esp_timer_get_time();
    if (r->stable) {
        bool changed = isnan(p->last_value) ||
                       fabsf(r->value - p->last_value) >= CHANGE_EPSILON;
        bool timed   = (now - p->last_stable_us) >=
                       (int64_t)STABLE_FORCE_MS * 1000;
        if (changed || timed) {
            meas_push(p->ch->id, SRC_SERIAL, Q_GOOD, r->value);
            p->last_value     = r->value;
            p->last_stable_us = now;
            if (changed) {
                /* weighing EVENT: ship it now, don't wait the upload tick */
                uplink_kick();
            }
        }
    } else if ((now - p->last_unstable_us) >= (int64_t)UNSTABLE_MIN_MS * 1000) {
        meas_push(p->ch->id, SRC_SERIAL, Q_UNSTABLE, r->value);
        p->last_unstable_us = now;
    }
}

static void reader_task(void *arg)
{
    sport_t *p = (sport_t *)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    char    line[SCALE_LINE_MAX];
    size_t  len = 0;
    uint8_t chunk[64];
    int64_t next_poll_us = 0;

    while (1) {
        /* request/response scales: send the poll command on schedule
         * (streaming scales have poll_cmd == "" and skip this) */
        if (p->ch->poll_cmd[0] != '\0' && p->ch->poll_period_ms > 0 &&
            esp_timer_get_time() >= next_poll_us) {
            uart_write_bytes(p->uart_num, p->ch->poll_cmd,
                             strlen(p->ch->poll_cmd));
            next_poll_us = esp_timer_get_time() +
                           (int64_t)p->ch->poll_period_ms * 1000;
        }

        int n = uart_read_bytes(p->uart_num, chunk, sizeof(chunk),
                                pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];
            if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0';
                    scale_reading_t r;
                    if (scale_parse_line(line, &r)) {
                        report(p, &r);
                    } else {
                        ESP_LOGD(TAG, "uart%d unparsed: '%s'",
                                 p->uart_num, line);
                    }
                    len = 0;
                }
            } else if (len < SCALE_LINE_MAX - 1) {
                line[len++] = c;
            } else {
                len = 0; /* oversize garbage: resync on next terminator */
            }
        }
    }
}

static esp_err_t port_start(sport_t *p, int port_no)
{
    const cfg_channel_t *ch = p->ch;
    const uart_config_t uc = {
        .baud_rate  = (int)ch->baud,
        .data_bits  = (ch->databits == 7) ? UART_DATA_7_BITS
                                          : UART_DATA_8_BITS,
        .parity     = (ch->parity == 'E') ? UART_PARITY_EVEN :
                      (ch->parity == 'O') ? UART_PARITY_ODD
                                          : UART_PARITY_DISABLE,
        .stop_bits  = (ch->stopbits == 2) ? UART_STOP_BITS_2
                                          : UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* small TX buffer so poll commands never block the reader */
    ESP_ERROR_CHECK(uart_driver_install(p->uart_num, RX_BUF_SIZE, 256,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(p->uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(p->uart_num, p->tx_gpio, p->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "cong %d: uart%d tx=%d rx=%d %u-%u%c%u parser=%s "
             "poll=%s/%ums (%s)", port_no, p->uart_num, p->tx_gpio,
             p->rx_gpio, (unsigned)ch->baud, (unsigned)ch->databits,
             ch->parity, (unsigned)ch->stopbits, ch->parser,
             ch->poll_cmd[0] ? "cmd" : "stream",
             (unsigned)ch->poll_period_ms, ch->code);

    char task_name[16];
    snprintf(task_name, sizeof(task_name), "scale_rx%d", port_no);
    BaseType_t ok = xTaskCreatePinnedToCore(reader_task, task_name, 3072,
                                            p, 8, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t scale_serial_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus != BUS_SERIAL) {
            continue;
        }
        /* JSON "uart": 1|2 -> s_ports[0|1]; moi cong nhan 1 kenh dau tien */
        int idx = (chans[i].uart == 2) ? 1 : 0;
        if (s_ports[idx].ch == NULL) {
            s_ports[idx].ch         = &chans[i];
            s_ports[idx].last_value = NAN;
        } else {
            ESP_LOGW(TAG, "%s: cong %d da co kenh %s — bo qua (moi cong "
                     "1 thiet bi)", chans[i].code, idx + 1,
                     s_ports[idx].ch->code);
        }
    }

    if (s_ports[0].ch == NULL && s_ports[1].ch == NULL) {
        ESP_LOGI(TAG, "no serial channel configured, reader not started");
        return ESP_OK;
    }

    for (int i = 0; i < 2; i++) {
        if (s_ports[i].ch != NULL) {
            esp_err_t err = port_start(&s_ports[i], i + 1);
            if (err != ESP_OK) {
                return err;
            }
        }
    }
    return ESP_OK;
}
