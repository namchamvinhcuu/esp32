#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "os/os_mbuf.h"

#include "cfg.h"
#include "meas_core.h"
#include "uplink.h"
#include "ble_central.h"

#define SCAN_WINDOW_S     30      /* mot luot quet */
#define RETRY_DELAY_MS    5000    /* nghi giua cac luot quet/ket noi lai */
#define LINE_BUF_MAX      64
#define SEEN_CACHE        8       /* chong spam log khi quet */

static const char *TAG = "blec";

/* UUID notify quen mat: FFE1 (module TQ kieu HM-10) va NUS TX cua Nordic */
static const ble_uuid16_t UUID_FFE1 = BLE_UUID16_INIT(0xFFE1);
static const ble_uuid128_t UUID_NUS_TX = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static const cfg_channel_t *s_ch;             /* kenh BLE dau tien */
static uint8_t   s_own_addr_type;
static uint16_t  s_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t  s_val_handle;                /* chr notify da chon */
static uint16_t  s_best_val_handle;           /* ung vien trong luc kham pha */
static bool      s_best_is_known;             /* ung vien la FFE1/NUS? */
static volatile bool s_subscribed;
static ble_addr_t s_seen[SEEN_CACHE];
static size_t    s_seen_n;

/* ---------------------------------------------------------------- parser */

/* raw_line: gom byte notify, gap \r/\n thi bat CUM SO dau tien trong dong
 * (mo hinh measureRegexp cu cua user / parser raw_line cua can) */
static char   s_line[LINE_BUF_MAX];
static size_t s_line_len;

static void push_value(float raw)
{
    float v = raw * s_ch->scale + s_ch->offset;
    meas_push(s_ch->id, SRC_BLE, Q_GOOD, v);
    uplink_kick(); /* thiet bi do = su kien: day len ngay */
    ESP_LOGI(TAG, "%s = %.3f", s_ch->code, (double)v);
}

static void parse_line(const char *line)
{
    for (const char *p = line; *p != '\0'; p++) {
        if (isdigit((unsigned char)*p) ||
            (*p == '-' && isdigit((unsigned char)p[1]))) {
            push_value(strtof(p, NULL));
            return;
        }
    }
}

static void parser_feed(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char ch = (char)data[i];
        if (ch == '\r' || ch == '\n') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                parse_line(s_line);
                s_line_len = 0;
            }
            continue;
        }
        if (s_line_len < LINE_BUF_MAX - 1) {
            s_line[s_line_len++] = ch;
        } else {
            /* khong thay xuong dong: giao thuc binary? — hexdump de con
             * nguoi viet parser rieng (nhu scale_parse) khi hang ve */
            ESP_LOG_BUFFER_HEX(TAG, s_line, s_line_len);
            s_line_len = 0;
        }
    }
}

/* ----------------------------------------------------------- target match */

static void addr_to_str(const ble_addr_t *a, char *out /*>=18*/)
{
    /* val[] la little-endian -> in nguoc cho giong dang nguoi doc */
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             a->val[5], a->val[4], a->val[3],
             a->val[2], a->val[1], a->val[0]);
}

static bool target_is_mac(void)
{
    return strlen(s_ch->host) == 17 && s_ch->host[2] == ':';
}

static bool match_target(const ble_addr_t *addr, const char *name)
{
    if (s_ch->host[0] == '\0') {
        return false; /* che do chi quet */
    }
    if (target_is_mac()) {
        char buf[18];
        addr_to_str(addr, buf);
        return strcasecmp(buf, s_ch->host) == 0;
    }
    return name[0] != '\0' &&
           strncasecmp(name, s_ch->host, strlen(s_ch->host)) == 0;
}

static bool seen_before(const ble_addr_t *a)
{
    for (size_t i = 0; i < s_seen_n; i++) {
        if (ble_addr_cmp(&s_seen[i], a) == 0) {
            return true;
        }
    }
    s_seen[s_seen_n % SEEN_CACHE] = *a;
    s_seen_n++;
    return false;
}

/* -------------------------------------------------------- GATT discovery */

static int desc_written_cb(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    if (error->status == 0) {
        s_subscribed = true;
        ESP_LOGI(TAG, "subscribed (val_handle=%u) — cho du lieu tu thiet bi",
                 (unsigned)s_val_handle);
    } else {
        ESP_LOGW(TAG, "ghi CCCD loi status=%d", error->status);
    }
    return 0;
}

static void subscribe_notify(void)
{
    /* CCCD thuong nam ngay sau value handle; du cho da so module do.
     * Thiet bi la doi thi refine khi hang ve (disc_all_dscs). */
    uint8_t on[2] = { 0x01, 0x00 };
    int rc = ble_gattc_write_flat(s_conn, s_val_handle + 1, on, sizeof(on),
                                  desc_written_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "write CCCD rc=%d", rc);
    }
}

static int chr_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_best_val_handle != 0) {
            s_val_handle = s_best_val_handle;
            subscribe_notify();
        } else {
            ESP_LOGW(TAG, "thiet bi khong co characteristic NOTIFY nao");
            ble_gap_terminate(s_conn, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }
    if (error->status != 0 || chr == NULL) {
        return 0;
    }
    if (!(chr->properties & BLE_GATT_CHR_PROP_NOTIFY)) {
        return 0;
    }
    char ubuf[BLE_UUID_STR_LEN];
    ESP_LOGI(TAG, "  chr notify: %s (val=%u)",
             ble_uuid_to_str(&chr->uuid.u, ubuf), (unsigned)chr->val_handle);
    bool known = ble_uuid_cmp(&chr->uuid.u, &UUID_FFE1.u) == 0 ||
                 ble_uuid_cmp(&chr->uuid.u, &UUID_NUS_TX.u) == 0;
    /* uu tien UUID quen mat; chua co gi thi lay ung vien dau tien */
    if (known || (!s_best_is_known && s_best_val_handle == 0)) {
        s_best_val_handle = chr->val_handle;
        s_best_is_known   = known;
    }
    return 0;
}

/* --------------------------------------------------------------- GAP     */

static int gap_event_cb(struct ble_gap_event *event, void *arg);

static void start_scan(void)
{
    struct ble_gap_disc_params p = {
        .passive = 1,
        .itvl    = 0,   /* stack default */
        .window  = 0,
    };
    s_seen_n = 0;
    int rc = ble_gap_disc(s_own_addr_type, SCAN_WINDOW_S * 1000, &p,
                          gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "scan start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "quet BLE %ds (muc tieu: %s)", SCAN_WINDOW_S,
                 s_ch->host[0] ? s_ch->host : "— chi liet ke");
    }
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields f;
        char name[32] = "";
        if (ble_hs_adv_parse_fields(&f, event->disc.data,
                                    event->disc.length_data) == 0 &&
            f.name != NULL && f.name_len > 0) {
            size_t n = f.name_len < sizeof(name) - 1 ? f.name_len
                                                     : sizeof(name) - 1;
            memcpy(name, f.name, n);
            name[n] = '\0';
        }
        if (!seen_before(&event->disc.addr)) {
            char abuf[18];
            addr_to_str(&event->disc.addr, abuf);
            ESP_LOGI(TAG, "thay %s rssi=%d name='%s'",
                     abuf, event->disc.rssi, name);
        }
        if (match_target(&event->disc.addr, name)) {
            ESP_LOGI(TAG, "khop muc tieu — ket noi...");
            ble_gap_disc_cancel();
            int rc = ble_gap_connect(s_own_addr_type, &event->disc.addr,
                                     10000, NULL, gap_event_cb, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "connect rc=%d", rc);
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            s_best_val_handle = 0;
            s_best_is_known   = false;
            ESP_LOGI(TAG, "da ket noi — kham pha characteristics...");
            ble_gattc_disc_all_chrs(s_conn, 1, 0xffff, chr_disc_cb, NULL);
        } else {
            ESP_LOGW(TAG, "ket noi loi status=%d", event->connect.status);
            s_conn = BLE_HS_CONN_HANDLE_NONE;
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "mat ket noi thiet bi (reason=%d) — se quet lai",
                 event->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed = false;
        s_line_len = 0;
        return 0;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint8_t buf[64];
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        ble_hs_mbuf_to_flat(event->notify_rx.om, buf, len, NULL);
        parser_feed(buf, len);
        return 0;
    }
    default:
        return 0;
    }
}

/* --------------------------------------------------------------- task    */

static void central_task(void *arg)
{
    /* ble_uart da init NimBLE; chi viec cho host len song */
    while (!ble_hs_synced()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "khong lay duoc dia chi BLE");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "central san sang — kenh '%s', muc tieu '%s', parser=%s",
             s_ch->code, s_ch->host[0] ? s_ch->host : "(quet)",
             s_ch->parser);

    while (1) {
        if (s_conn == BLE_HS_CONN_HANDLE_NONE && !ble_gap_disc_active()) {
            start_scan();
        }
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }
}

esp_err_t ble_central_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    size_t n_ble = 0;
    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus == BUS_BLE) {
            if (s_ch == NULL) {
                s_ch = &chans[i];
            }
            n_ble++;
        }
    }
    if (s_ch == NULL) {
        ESP_LOGI(TAG, "khong co kenh BLE, central khong chay");
        return ESP_OK;
    }
    if (n_ble > 1) {
        ESP_LOGW(TAG, "v1 chi ho tro 1 thiet bi BLE — dung kenh '%s', "
                 "%u kenh BLE khac bi bo qua", s_ch->code,
                 (unsigned)(n_ble - 1));
    }

    BaseType_t ok = xTaskCreatePinnedToCore(central_task, "ble_central",
                                            4096, NULL, 4, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
