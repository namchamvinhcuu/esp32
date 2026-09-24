#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cfg.h"
#include "gpio_out.h"
#include "meas_core.h"

static const char *TAG = "gpio_out";

/* Nhip cua tac vu. 50 ms la du min cho mat nguoi (chu ky chop ngan nhat
 * dung duoc la ~200 ms) va van re: mot vong lap qua 32 phan tu. */
#define TICK_MS 50

/* Nguoi bam tay thi giu bao lau truoc khi tra ve cho canh bao tu dong.
 *
 * Phai co han. Khong han thi mot lan thu den luc 10 gio sang se lam den
 * bao dong cua ca day chuyen chet im tu do tro di, va khong ai nho de tra
 * lai. Co han thi cai gia cua viec quen chi la 5 phut. */
#define MANUAL_HOLD_MS (5 * 60 * 1000)

/* Nhac lai trang thai dinh ky. Khong phai de dieu khien — chan GPIO khong
 * tu quen — ma de ben Odoo va trang /ops khong bi "oi": bao-khi-doi nghia
 * la kenh im lang mai neu khong ai cham vao no. */
#define REPORT_EVERY_MS 30000

/* Che do nhan dang: sang lan luot tung kenh de nguoi o xuong doi chieu
 * chan nao ra bong mau gi. 3 giay mot kenh la du de nhin va ghi lai. */
#define IDENT_STEP_MS 3000

enum { PAT_STEADY = 0, PAT_BLINK = 1 };

typedef struct {
    bool     configured;
    uint8_t  pattern;
    int      level;          /* muc logic dang phat (1 = bat) */
    int64_t  expire_us;      /* 0 = giu mai */
    int64_t  manual_us;      /* toi luc nay, tu dong khong duoc cham */
    uint32_t period_ms;
    int64_t  next_flip_us;
    int64_t  next_report_us;
} out_t;

static out_t s_out[CFG_MAX_CHANNELS];

/* Che do nhan dang: -1 = khong chay, con lai la chi so kenh dang sang. */
static int     s_ident = -1;
static int64_t s_ident_next_us;

static inline int out_level(const cfg_channel_t *ch, int on)
{
    return ch->invert ? !on : on;
}

/* Dat muc ra chan va bao len may chu NEU co doi. `force_report` dung cho
 * cac moc dinh ky. */
static void drive(const cfg_channel_t *ch, int on, bool force_report)
{
    out_t *o = &s_out[ch->id];
    bool changed = (o->level != on);
    o->level = on;
    gpio_set_level(ch->gpio_pin, out_level(ch, on));
    if (changed || force_report) {
        /* Giu anh chup ben Odoo dung voi cai ta THUC SU da ghi ra chan,
         * khong phai cai duoc yeu cau — io_scan.c lam y het cho dau vao. */
        meas_push(ch->id, SRC_GPIO, Q_GOOD, (float)on);
        o->next_report_us = esp_timer_get_time() + (int64_t)REPORT_EVERY_MS * 1000;
    }
}

static const cfg_channel_t *chan_by_pin(int pin)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus == BUS_GPIO && chans[i].io_mode == IO_OUTPUT &&
            chans[i].gpio_pin == pin && s_out[chans[i].id].configured) {
            return &chans[i];
        }
    }
    return NULL;
}

/* ── tac vu nhip ────────────────────────────────────────────────────────
 *
 * Chop KHONG day mot ban ghi nao len may chu.
 *
 * Co chu dich: mot den chop 500 ms ma bao moi lan dao muc thi la 4 ban ghi
 * moi giay tren mot kenh, cong voi trigger ben Odoo an theo — dung cai
 * hinh dang tai da lam sap instance ngay 18/09 (2891 lan kich trong 6 gio,
 * trung vi buoc thay doi 0,0000 g). Ta bao "kenh nay dang chop", mot lan,
 * luc bat dau va luc ket thuc. Chi tiet tung nhip la viec cua chan dien.
 */
static void gpio_out_task(void *arg)
{
    (void)arg;
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        const int64_t now = esp_timer_get_time();

        /* nhan dang: sang lan luot */
        if (s_ident >= 0 && now >= s_ident_next_us) {
            while (s_ident < (int)count) {
                const cfg_channel_t *c = &chans[s_ident];
                if (c->bus == BUS_GPIO && c->io_mode == IO_OUTPUT &&
                    s_out[c->id].configured) {
                    break;
                }
                s_ident++;
            }
            /* tat cai vua sang */
            for (size_t i = 0; i < count; i++) {
                if (s_out[chans[i].id].configured && s_out[chans[i].id].level) {
                    drive(&chans[i], 0, false);
                }
            }
            if (s_ident >= (int)count) {
                ESP_LOGW(TAG, "nhan dang: xong");
                s_ident = -1;
            } else {
                const cfg_channel_t *c = &chans[s_ident];
                ESP_LOGW(TAG, ">>> nhan dang: %s (gpio %d) DANG SANG <<<",
                         c->code, (int)c->gpio_pin);
                s_out[c->id].pattern   = PAT_STEADY;
                s_out[c->id].expire_us = 0;
                s_out[c->id].manual_us = now + (int64_t)IDENT_STEP_MS * 1000;
                drive(c, 1, true);
                s_ident++;
                s_ident_next_us = now + (int64_t)IDENT_STEP_MS * 1000;
            }
        }

        for (size_t i = 0; i < count; i++) {
            const cfg_channel_t *ch = &chans[i];
            out_t *o = &s_out[ch->id];
            if (!o->configured) {
                continue;
            }
            if (o->expire_us != 0 && now >= o->expire_us) {
                o->pattern   = PAT_STEADY;
                o->expire_us = 0;
                drive(ch, 0, true);       /* het gio: tat, va bao mot lan */
                continue;
            }
            if (o->pattern == PAT_BLINK && now >= o->next_flip_us) {
                o->next_flip_us = now + (int64_t)(o->period_ms / 2) * 1000;
                /* dao muc THANG ra chan, khong qua drive(): khong bao len
                 * may chu tung nhip — xem ghi chu dau ham. */
                o->level = !o->level;
                gpio_set_level(ch->gpio_pin, out_level(ch, o->level));
                continue;
            }
            if (now >= o->next_report_us) {
                drive(ch, o->level, true);   /* nhac lai dinh ky */
            }
        }
    }
}

esp_err_t gpio_out_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    uint64_t mask = 0;
    size_t n = 0;

    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus == BUS_GPIO && chans[i].io_mode == IO_OUTPUT) {
            mask |= 1ULL << chans[i].gpio_pin;
            n++;
        }
    }
    if (n == 0) {
        ESP_LOGI(TAG, "khong co kenh gpio output nao");
        return ESP_OK;
    }

    gpio_config_t gc = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&gc);
    if (err != ESP_OK) {
        return err;
    }

    const int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < count; i++) {
        const cfg_channel_t *ch = &chans[i];
        if (ch->bus != BUS_GPIO || ch->io_mode != IO_OUTPUT) {
            continue;
        }
        out_t *o = &s_out[ch->id];
        o->configured    = true;
        o->pattern       = PAT_STEADY;
        o->level         = 0;
        o->next_report_us = now + (int64_t)REPORT_EVERY_MS * 1000;
        gpio_set_level(ch->gpio_pin, out_level(ch, 0));   /* tat luc khoi dong */
        ESP_LOGI(TAG, "%s: gpio%d output (invert=%d)", ch->code, ch->gpio_pin,
                 (int)ch->invert);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(gpio_out_task, "gpio_out", 2560,
                                            NULL, 3, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

bool gpio_out_auto_write_pin(int pin, int on)
{
    const cfg_channel_t *ch = chan_by_pin(pin);
    if (ch == NULL) {
        return false;           /* khong phai chan cua ta — ben goi tu lo */
    }
    out_t *o = &s_out[ch->id];
    if (esp_timer_get_time() < o->manual_us) {
        return true;            /* dang do nguoi giu — tu dong nhuong */
    }
    if (o->pattern != PAT_STEADY || o->expire_us != 0) {
        return true;            /* dang chay mot mau phat — dung cat ngang */
    }
    drive(ch, on ? 1 : 0, false);
    return true;
}

bool gpio_out_execute(const gpio_cmd_t *cmd, char *detail, size_t detail_sz)
{
    const char *op = (cmd->op != NULL) ? cmd->op : "";

    if (strcmp(op, "identify") == 0) {
        s_ident = 0;
        s_ident_next_us = 0;     /* chay ngay o nhip ke tiep */
        ESP_LOGW(TAG, "nhan dang: bat dau, moi kenh %d ms", IDENT_STEP_MS);
        detail[0] = '\0';
        return true;
    }

    const cfg_channel_t *ch = cfg_channel_by_code(cmd->channel);
    if (ch == NULL) {
        snprintf(detail, detail_sz, "khong biet kenh '%s'",
                 (cmd->channel && cmd->channel[0]) ? cmd->channel : "?");
        return false;
    }
    if (ch->bus != BUS_GPIO || ch->io_mode != IO_OUTPUT) {
        snprintf(detail, detail_sz, "kenh '%s' khong phai GPIO output — "
                 "firmware nay chua ho tro dieu khien tu xa cho no", cmd->channel);
        return false;
    }
    out_t *o = &s_out[ch->id];
    if (!o->configured) {
        snprintf(detail, detail_sz, "kenh '%s' chua duoc cau hinh", cmd->channel);
        return false;
    }

    const int64_t now = esp_timer_get_time();
    /* Moi lenh tu may chu deu la "nguoi dieu khien": giu quyen MANUAL_HOLD_MS
     * roi tra ve cho canh bao tu dong. */
    o->manual_us = now + (int64_t)MANUAL_HOLD_MS * 1000;

    if (strcmp(op, "blink") == 0) {
        uint32_t period = (cmd->period_ms > 0) ? (uint32_t)cmd->period_ms : 1000;
        if (period < 100) {
            period = 100;       /* duoi nguong nay mat khong theo kip, va
                                 * tac vu nhip 50 ms cung khong chia duoc */
        }
        o->pattern      = PAT_BLINK;
        o->period_ms    = period;
        o->next_flip_us = now + (int64_t)(period / 2) * 1000;
        o->expire_us    = (cmd->ms > 0) ? now + (int64_t)cmd->ms * 1000 : 0;
        drive(ch, 1, true);
        detail[0] = '\0';
        return true;
    }

    int level;
    if (strcmp(op, "write") == 0) {
        level = cmd->has_value ? (cmd->value != 0) : 0;
    } else if (strcmp(op, "on") == 0) {
        level = 1;
    } else if (strcmp(op, "off") == 0) {
        level = 0;
    } else {
        snprintf(detail, detail_sz,
                 "kenh GPIO output chi nhan: write | on | off | blink | identify");
        return false;
    }

    o->pattern   = PAT_STEADY;
    o->period_ms = 0;
    /* Hen gio chi co nghia khi BAT. "Tat trong 10 giay roi tu bat lai" la
     * thu khong ai muon o mot cai den bao. */
    o->expire_us = (level && cmd->ms > 0) ? now + (int64_t)cmd->ms * 1000 : 0;
    drive(ch, level, true);
    detail[0] = '\0';
    return true;
}
