#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "cfg.h"
#include "io_scan.h"
#include "meas_core.h"
#include "uplink.h"

#define MAX_IO      16   /* kenh GPIO toi da (count+state moi chan = 2 kenh) */
#define STATE_SCAN_MS 50

static const char *TAG = "ioscan";

typedef struct {
    const cfg_channel_t *ch;
    /* COUNT mode (ISR context) */
    volatile uint32_t count;
    volatile int64_t  lockout_until_us;
    uint32_t reported_count; /* gia tri da bao lan cuoi (task context) */
    /* STATE mode */
    int      stable_level;   /* debounced level, -1 = unknown yet */
    int      cand_level;
    int64_t  cand_since_us;
    /* COUNT mode: chi dem khi chan da tro ve muc nghi ke tu lan dem truoc.
     * Xem chu thich o pulse_isr(). */
    volatile bool armed;
    int      idle_level;     /* muc nghi, -1 = khong chan (edge=both) */
    /* reporting */
    int64_t  next_report_us;
} io_chan_t;

static io_chan_t s_io[MAX_IO];
static size_t    s_io_count;

/* ISR: count edges with a time lockout as debounce. Runs in IRAM-safe
 * default ISR service; esp_timer_get_time() is ISR-safe.
 *
 * 2026-09-21 — vi sao co them `armed`: do that tren ban dap that, sau khi
 * count1 chuyen sang dung chan cua ban dap:
 *
 *     pedal1 (STATE) thay 6 lan dap  ->  count1 (COUNT) dem 9
 *
 * Nguyen nhan la NHA chan chu khong phai dap. Nguoi ta giu ban dap khoang
 * mot giay; luc nha ra tiep diem rung, va moi con rung do la mot suon
 * xuong moi — nam NGOAI cua so khoa 40 ms da mo tu luc bam. Nang
 * debounce_ms len khong chua duoc: cua so phai dai hon ca cu bam thi moi
 * trum het, ma nhu vay thi hai lan dap that lien nhau se bi gop lam mot.
 *
 * Nen dem theo CHU KY chu khong theo thoi gian: mot suon chi duoc dem khi
 * chan da tro ve muc nghi ke tu lan dem truoc. Bam - nha - bam la hai, con
 * rung bao nhieu lan trong cung mot cu bam cung chi la mot. Lockout thoi
 * gian van giu nguyen cho con rung luc BAM. */
static void IRAM_ATTR pulse_isr(void *arg)
{
    io_chan_t *io = arg;
    int64_t now = esp_timer_get_time();
    if (!io->armed) {
        return;
    }
    if (now >= io->lockout_until_us) {
        io->count++;
        io->armed = (io->idle_level < 0);   /* edge=both: khong chan chu ky */
        io->lockout_until_us = now +
            (int64_t)io->ch->debounce_ms * 1000;
    }
}

static int read_level(const io_chan_t *io)
{
    int lvl = gpio_get_level(io->ch->gpio_pin);
    return io->ch->invert ? !lvl : lvl;
}

static void scan_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(STATE_SCAN_MS));
        esp_task_wdt_reset();
        int64_t now = esp_timer_get_time();

        for (size_t i = 0; i < s_io_count; i++) {
            io_chan_t *io = &s_io[i];

            if (io->ch->io_mode == IO_STATE) {
                /* level must hold for debounce_ms before it counts */
                int lvl = read_level(io);
                if (lvl != io->cand_level) {
                    io->cand_level = lvl;
                    io->cand_since_us = now;
                }
                bool held = (now - io->cand_since_us) >=
                            (int64_t)io->ch->debounce_ms * 1000;
                if (held && lvl != io->stable_level) {
                    io->stable_level = lvl;
                    meas_push(io->ch->id, SRC_GPIO, Q_GOOD, (float)lvl);
                    io->next_report_us = now +
                        (int64_t)io->ch->period_ms * 1000;
                    /* su kien (dap/nha ban dap, den quang doi trang thai):
                     * danh thuc uplink day len NGAY thay vi cho nhip
                     * upload 10s — cung co che voi lan can cua scale */
                    uplink_kick();
                }
            }

            /* COUNT: bao NGAY khi so dem nhich (nhip quet 50ms) + kick —
             * san pham qua cam bien la len card sau ~2-4s thay vi cho
             * chuyen upload 10s (trung binh 5s). Toc do day toi da bi
             * chan boi nhip quet (20 mau/s) — du cho chuyen san xuat;
             * encoder kHz thi khong dung duong nay. */
            if (io->ch->io_mode == IO_COUNT) {
                /* Nap lai khi chan da ve muc nghi va het cua so khoa —
                 * cung nhip quet 50 ms voi STATE. */
                if (!io->armed && io->idle_level >= 0 &&
                    now >= io->lockout_until_us &&
                    gpio_get_level(io->ch->gpio_pin) == io->idle_level) {
                    io->armed = true;
                }
                uint32_t c = io->count;
                if (c != io->reported_count) {
                    io->reported_count = c;
                    meas_push(io->ch->id, SRC_GPIO, Q_GOOD,
                              (float)c * io->ch->scale + io->ch->offset);
                    io->next_report_us = now +
                        (int64_t)io->ch->period_ms * 1000;
                    uplink_kick();
                }
            }

            /* periodic report: COUNT sends the running total, STATE
             * re-sends the current level as a keep-alive */
            if (now >= io->next_report_us) {
                float v = (io->ch->io_mode == IO_COUNT)
                              ? (float)io->count
                              : (float)(io->stable_level < 0 ? read_level(io)
                                                             : io->stable_level);
                if (io->ch->io_mode == IO_COUNT) {
                    io->reported_count = io->count;
                }
                meas_push(io->ch->id, SRC_GPIO, Q_GOOD,
                          v * io->ch->scale + io->ch->offset);
                io->next_report_us = now + (int64_t)io->ch->period_ms * 1000;
            }
        }
    }
}

esp_err_t io_scan_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    for (size_t i = 0; i < count && s_io_count < MAX_IO; i++) {
        /* OUTPUT channels (relay/lamp) belong to gpio_out, not here — this
         * component only ever drives pins as INPUT. */
        if (chans[i].bus == BUS_GPIO && chans[i].io_mode != IO_OUTPUT) {
            s_io[s_io_count].ch = &chans[i];
            s_io[s_io_count].stable_level = -1;
            s_io[s_io_count].cand_level = -1;
            /* Suon xuong -> muc nghi la cao, va nguoc lai. edge=both thi
             * khong co "muc nghi" nao ca: dem moi suon nhu truoc. */
            s_io[s_io_count].idle_level =
                (chans[i].edge == EDGE_RISING) ? 0 :
                (chans[i].edge == EDGE_BOTH)   ? -1 : 1;
            s_io[s_io_count].armed = true;
            s_io_count++;
        }
    }
    if (s_io_count == 0) {
        ESP_LOGI(TAG, "no gpio channels configured");
        return ESP_OK;
    }

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { /* already installed */
        return err;
    }

    for (size_t i = 0; i < s_io_count; i++) {
        io_chan_t *io = &s_io[i];
        const cfg_channel_t *c = io->ch;

        /* Mot chan GPIO duoc phep mang NHIEU kenh (vd COUNT + STATE cung
         * chan: dem xung VA theo doi muc — quang dem san pham + "den dang
         * sang", ban dap trang thai + dem lan dap). gpio_config chi goi
         * MOT LAN cho moi chan, kieu ngat lay theo kenh COUNT neu co;
         * truoc day kenh cau hinh SAU de len kenh truoc (STATE sau COUNT
         * lam tat ngat dem xung). Gioi han: toi da 1 kenh COUNT/chan. */
        bool pin_done = false;
        for (size_t j = 0; j < i; j++) {
            if (s_io[j].ch->gpio_pin == c->gpio_pin) {
                pin_done = true;
                break;
            }
        }
        if (!pin_done) {
            const cfg_channel_t *cnt = NULL;
            for (size_t j = 0; j < s_io_count; j++) {
                if (s_io[j].ch->gpio_pin == c->gpio_pin &&
                    s_io[j].ch->io_mode == IO_COUNT) {
                    cnt = s_io[j].ch;
                    break;
                }
            }
            gpio_config_t gc = {
                .pin_bit_mask = 1ULL << c->gpio_pin,
                .mode         = GPIO_MODE_INPUT,
                .pull_up_en   = c->pull_down ? GPIO_PULLUP_DISABLE
                                             : GPIO_PULLUP_ENABLE,
                .pull_down_en = c->pull_down ? GPIO_PULLDOWN_ENABLE
                                             : GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_DISABLE,
            };
            if (cnt != NULL) {
                gc.intr_type = (cnt->edge == EDGE_RISING) ? GPIO_INTR_POSEDGE :
                               (cnt->edge == EDGE_BOTH)   ? GPIO_INTR_ANYEDGE
                                                          : GPIO_INTR_NEGEDGE;
            }
            ESP_ERROR_CHECK(gpio_config(&gc));
        }
        if (c->io_mode == IO_COUNT) {
            ESP_ERROR_CHECK(gpio_isr_handler_add(c->gpio_pin, pulse_isr, io));
        }
        ESP_LOGI(TAG, "%s: gpio%d %s debounce=%ums period=%ums",
                 c->code, c->gpio_pin,
                 c->io_mode == IO_COUNT ? "count" : "state",
                 (unsigned)c->debounce_ms, (unsigned)c->period_ms);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(scan_task, "io_scan", 2560,
                                            NULL, 8, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
