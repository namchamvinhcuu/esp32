#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "cfg.h"
#include "meas_core.h"
#include "gpio_out.h"
#include "mb_rtu.h"
#include "net_mgr.h"
#include "tower_light.h"
#include "uplink.h"

static const char *TAG = "tower";

typedef struct { int r, y, g, b, bz; } tower_state_t;

static tower_state_t s_override = { -1, -1, -1, -1, -1 };
static int64_t       s_override_until_us;
static tower_state_t s_shown = { -1, -1, -1, -1, -1 };

#if CONFIG_FMS_TOWER_BACKEND_GPIO
static const gpio_num_t PINS[5] = {
    CONFIG_FMS_TOWER_R_GPIO, CONFIG_FMS_TOWER_Y_GPIO,
    CONFIG_FMS_TOWER_G_GPIO, CONFIG_FMS_TOWER_B_GPIO,
    CONFIG_FMS_TOWER_BZ_GPIO,
};

/* Xin gpio_out ghi ho truoc.
 *
 * Bon trong nam chan nay TRUNG voi kenh relay cua Odoo (5/6/7/14), va ten
 * hai ben xao nhau: GPIO 5 o day la "do" nhung Odoo goi la relay_yellow.
 * Truoc 19/09/2026 ham nay ghi thang gpio_set_level(), va vong lap 10 giay
 * ben duoi ep ghi lai — nen mot den bat qua Odoo tu tat trong vong 10 giay,
 * am tham, khong de lai mot ban ghi nao. Do la loi "den sang mot luc roi
 * tat" ma khong ai giai thich duoc.
 *
 * Gio gpio_out la chu so huu: no tu choi ta khi kenh dang do nguoi giu, va
 * moi thay doi deu duoc bao len may chu. Chi chan khong thuoc kenh nao moi
 * roi xuong duong ghi thang o duoi. */
static void write_pin(gpio_num_t pin, int on)
{
    if (gpio_out_auto_write_pin((int)pin, on)) {
        return;
    }
#if CONFIG_FMS_TOWER_ACTIVE_LOW
    gpio_set_level(pin, on ? 0 : 1); /* opto relay boards trigger LOW */
#else
    gpio_set_level(pin, on ? 1 : 0);
#endif
}

static bool backend_write(const tower_state_t *st)
{
    write_pin(PINS[0], st->r);
    write_pin(PINS[1], st->y);
    write_pin(PINS[2], st->g);
    write_pin(PINS[3], st->b);
    write_pin(PINS[4], st->bz);
    return true;
}
#else /* CONFIG_FMS_TOWER_BACKEND_MBRTU */
/* Waveshare Modbus RTU Relay class module on the RS485 bus:
 * coils COIL_BASE..+4 = R, Y, G, B, BZ */
static bool backend_write(const tower_state_t *st)
{
    uint8_t bits = (uint8_t)((st->r  ? 0x01 : 0) |
                             (st->y  ? 0x02 : 0) |
                             (st->g  ? 0x04 : 0) |
                             (st->b  ? 0x08 : 0) |
                             (st->bz ? 0x10 : 0));
    return mb_rtu_write_coils(CONFIG_FMS_TOWER_MB_SLAVE,
                              CONFIG_FMS_TOWER_MB_COIL_BASE,
                              5, &bits) == ESP_OK;
}
#endif

/* absent/unpowered relay module: warn once, retry gently, recover quietly */
#define MODULE_RETRY_US (10 * 1000000LL)
static bool    s_module_down;
static int64_t s_retry_after_us;

static void apply(tower_state_t st, bool force)
{
    if (!force && memcmp(&st, &s_shown, sizeof(st)) == 0) {
        return;
    }
    /* module unreachable: back off instead of hammering the bus + log */
    int64_t now = esp_timer_get_time();
    if (now < s_retry_after_us) {
        return;
    }

    if (backend_write(&st)) {
        if (s_module_down) {
            ESP_LOGI(TAG, "relay module responding — tower output restored");
            s_module_down = false;
        }
        if (memcmp(&st, &s_shown, sizeof(st)) != 0) {
            /* ha xuong D 21/07 (yeu cau user - console sach): bat lai khi
             * can debug cuc tinh bang esp_log_level_set("tower", DEBUG) */
            ESP_LOGD(TAG, "R=%d Y=%d G=%d B=%d BZ=%d",
                     st.r, st.y, st.g, st.b, st.bz);
        }
        s_shown = st;
    } else {
        /* silent by default — the config page shows this state instead */
        if (!s_module_down) {
            ESP_LOGD(TAG, "relay module not responding; retrying every 10 s");
            s_module_down = true;
        }
        s_retry_after_us = now + MODULE_RETRY_US;
    }
}

bool tower_output_ok(void)
{
    return !s_module_down;
}

const char *tower_backend_name(void)
{
#if !CONFIG_FMS_TOWER_ENABLE
    return "off";
#elif CONFIG_FMS_TOWER_BACKEND_GPIO
    return "gpio";
#else
    return "mbrtu";
#endif
}

static tower_state_t eval_local(void)
{
    tower_state_t st = { 0, 0, 0, 0, 0 };

    bool any_alarm = false, any_comm_err = false;
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    for (size_t i = 0; i < count; i++) {
        measurement_t m;
        if (!meas_latest(chans[i].id, &m)) {
            continue;
        }
        if (m.quality == Q_COMM_ERR) {
            any_comm_err = true;
        } else if (m.quality == Q_GOOD) {
            if ((!isnan(chans[i].alarm_high) && m.value > chans[i].alarm_high) ||
                (!isnan(chans[i].alarm_low)  && m.value < chans[i].alarm_low)) {
                any_alarm = true;
            }
        }
    }

    st.r = any_alarm;
#if CONFIG_FMS_TOWER_STATUS_LIGHTS
    /* kieu "den trang thai": xanh = khoe, vang = mat mang/server/loi doc */
    EventBits_t bits = net_mgr_bits();
    bool net_ok = (bits & NET_BIT_WIFI) && (bits & NET_BIT_TIME);
    bool srv_ok = uplink_server_ok();
    st.y = (any_comm_err || !net_ok || !srv_ok);
    st.g = (net_ok && srv_ok && !any_alarm);
#else
    /* che do IM LANG (mac dinh tu 21/07/2026): den tat la binh thuong —
     * chi sang DO khi alarm, hoac theo lenh dieu khien tu server (override).
     * any_comm_err van duoc tinh de khoi canh bao unused khi doi mode. */
    (void)any_comm_err;
#endif
#if CONFIG_FMS_TOWER_BUZZER_ON_ALARM
    st.bz = any_alarm;
#endif
    return st;
}

static void tower_task(void *arg)
{
    int refresh = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        tower_state_t st = eval_local();
        if (esp_timer_get_time() < s_override_until_us) {
            /* server override wins per-color; -1 keeps the local value */
            if (s_override.r  >= 0) st.r  = s_override.r;
            if (s_override.y  >= 0) st.y  = s_override.y;
            if (s_override.g  >= 0) st.g  = s_override.g;
            if (s_override.b  >= 0) st.b  = s_override.b;
            if (s_override.bz >= 0) st.bz = s_override.bz;
        }
        /* re-assert every ~10 s: a power-cycled Modbus relay module wakes
         * with all coils off and would otherwise stay dark until a change */
        bool force = (++refresh >= 20);
        if (force) {
            refresh = 0;
        }
        apply(st, force);
    }
}

void tower_override(int r, int y, int g, int b, int bz)
{
    s_override = (tower_state_t){ r, y, g, b, bz };
    s_override_until_us = esp_timer_get_time() +
                          (int64_t)TOWER_OVERRIDE_TTL_MS * 1000;
}

esp_err_t tower_light_start(void)
{
#if !CONFIG_FMS_TOWER_ENABLE
    ESP_LOGI(TAG, "disabled in menuconfig");
    return ESP_OK;
#else
#if CONFIG_FMS_TOWER_BACKEND_GPIO
    uint64_t mask = 0;
    for (size_t i = 0; i < 5; i++) {
        mask |= 1ULL << PINS[i];
    }
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "backend: GPIO relays %d/%d/%d/%d/%d",
             PINS[0], PINS[1], PINS[2], PINS[3], PINS[4]);
#else
    ESP_LOGI(TAG, "backend: Modbus RTU relay module addr=%d coils %d..%d",
             CONFIG_FMS_TOWER_MB_SLAVE, CONFIG_FMS_TOWER_MB_COIL_BASE,
             CONFIG_FMS_TOWER_MB_COIL_BASE + 4);
#endif
    apply((tower_state_t){ 0, 0, 0, 0, 0 }, true); /* everything off at boot */
    s_shown = (tower_state_t){ -1, -1, -1, -1, -1 };

    uplink_set_tower_cb(tower_override); /* server "tower" commands */

    BaseType_t ok = xTaskCreatePinnedToCore(tower_task, "tower", 2048,
                                            NULL, 4, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}
