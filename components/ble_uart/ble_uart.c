#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

#include "cfg.h"
#include "meas_core.h"
#include "ble_uart.h"

static const char *TAG = "bleuart";

/* Nordic UART Service UUIDs (bytes little-endian for BLE_UUID128_INIT):
 *   service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX      6E400002-... (client writes)
 *   TX      6E400003-... (server notifies)                                 */
static const ble_uuid128_t UUID_SVC = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t UUID_RX = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t UUID_TX = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle;
static volatile bool s_subscribed;
static char s_dev_name[16];

static void start_advertising(void);

/* RX: log what the client sends (command channel reserved for later) */
static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char buf[64];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(buf)) {
            len = sizeof(buf) - 1;
        }
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        buf[len] = '\0';
        ESP_LOGI(TAG, "rx from client: '%s'", buf);
        return 0;
    }
    /* TX is notify-only; reads return empty */
    return 0;
}

static const struct ble_gatt_svc_def GATT_SVCS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SVC.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &UUID_RX.u,
                .access_cb  = gatt_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = &UUID_TX.u,
                .access_cb  = gatt_access_cb,
                .val_handle = &s_tx_val_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "client connected");
        } else {
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "client disconnected (reason %d)",
                 event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed  = false;
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            s_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "notifications %s",
                     s_subscribed ? "enabled" : "disabled");
        }
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu = %u", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = { 0 };
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)s_dev_name;
    fields.name_len = strlen(s_dev_name);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "advertising as %s", s_dev_name);
    }
}

static void on_sync(void)
{
    uint8_t own_addr_type;
    if (ble_hs_util_ensure_addr(0) != 0 ||
        ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        ESP_LOGE(TAG, "no BLE address available");
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset, reason=%d", reason);
}

static void host_task(void *arg)
{
    nimble_port_run();            /* returns on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* Send one buffer, chunked to the connection's ATT payload (MTU-3). */
static bool notify_chunked(const char *data, size_t len)
{
    uint16_t mtu = ble_att_mtu(s_conn_handle);
    size_t chunk_max = (mtu > 23 ? mtu : 23) - 3;
    while (len > 0) {
        size_t n = len < chunk_max ? len : chunk_max;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data, n);
        if (om == NULL) {
            return false;
        }
        int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
        if (rc != 0) {
            return false; /* om consumed by the stack on both paths */
        }
        data += n;
        len  -= n;
    }
    return true;
}

/* 1 Hz: push every channel whose latest measurement changed since the
 * previous round. seq==0 means "not sent yet".
 * Phien subscribe MOI: gui 1 dong META (node/serial/loc) + phat lai TAT CA
 * kenh mot luot (app reconnect thay ngay du danh sach, khong cho doi gia
 * tri). Moi dong kenh mang "a" = co canh bao (vuot alarm_high/low). */
static void feed_task(void *arg)
{
    static uint32_t last_sent_seq[CFG_MAX_CHANNELS];
    bool was_subscribed = false;
    bool meta_sent = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_subscribed || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            was_subscribed = false;
            continue;
        }
        if (!was_subscribed) {
            memset(last_sent_seq, 0, sizeof(last_sent_seq));
            meta_sent = false;
            was_subscribed = true;
        }
        if (!meta_sent) {
            char loc[48];
            strlcpy(loc, cfg_loc_name(), sizeof(loc));
            for (char *p = loc; *p != '\0'; p++) {
                if (*p == '"' || *p == '\\') {
                    *p = '\'';
                }
            }
            char meta[160];
            int mn = snprintf(meta, sizeof(meta),
                              "{\"node\":\"%s\",\"serial\":\"%s\","
                              "\"loc\":\"%s\",\"fw\":\"0.1.0\"}\n",
                              s_dev_name, cfg_node_serial(), loc);
            if (mn <= 0 || mn >= (int)sizeof(meta) ||
                !notify_chunked(meta, (size_t)mn)) {
                continue; /* nghen: thu lai vong sau */
            }
            meta_sent = true;
        }

        size_t count = 0;
        const cfg_channel_t *chans = cfg_get_channels(&count);
        for (size_t i = 0; i < count && i < CFG_MAX_CHANNELS; i++) {
            measurement_t m;
            if (!meas_latest(chans[i].id, &m) || m.seq == last_sent_seq[i]) {
                continue;
            }
            const cfg_channel_t *c = &chans[i];
            bool alarm =
                (!isnan(c->alarm_high) && m.value > c->alarm_high) ||
                (!isnan(c->alarm_low)  && m.value < c->alarm_low);
            char line[184];
            /* "u"=don vi, "n"=ten, "a"=canh bao — app cu bo qua key la */
            int n = snprintf(line, sizeof(line),
                             "{\"ch\":\"%s\",\"v\":%.3f,\"q\":%u,"
                             "\"ts\":%lld,\"u\":\"%s\",\"n\":\"%s\","
                             "\"a\":%d}\n",
                             c->code, (double)m.value,
                             (unsigned)m.quality, (long long)m.ts_ms,
                             c->unit_name, c->name, alarm ? 1 : 0);
            if (n <= 0 || n >= (int)sizeof(line)) {
                continue;
            }
            if (notify_chunked(line, (size_t)n)) {
                last_sent_seq[i] = m.seq;
            } else {
                break; /* congested/disconnected: retry next round */
            }
        }
    }
}

esp_err_t ble_uart_start(void)
{
    /* "FMS-" + last 4 hex of the node serial, e.g. FMS-B2C4 */
    const char *serial = cfg_node_serial();
    size_t sl = strlen(serial);
    snprintf(s_dev_name, sizeof(s_dev_name), "FMS-%s",
             sl >= 4 ? serial + sl - 4 : serial);

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(GATT_SVCS);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(GATT_SVCS);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "gatt registration rc=%d", rc);
        return ESP_FAIL;
    }
    ble_svc_gap_device_name_set(s_dev_name);

    nimble_port_freertos_init(host_task);

    BaseType_t ok = xTaskCreatePinnedToCore(feed_task, "ble_feed", 3072,
                                            NULL, 4, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
