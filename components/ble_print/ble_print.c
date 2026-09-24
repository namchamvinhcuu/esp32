#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"

#include "ble_print.h"

#define MAX_PRINTERS     4      /* pool; NIMBLE_MAX_CONNECTIONS=6 phai du:
                                   1 dien thoai + 1 thuoc kep + 4 may in */
#define SCAN_MS          8000   /* lan dau chua biet addr type: phai quet */
#define CONNECT_MS       5000   /* da biet dia chi: ket noi thang */
#define SETUP_TIMEOUT_MS 20000
#define CHUNK_RETRY_MAX  100

#define NVS_NS  "bleprint"
#define NVS_KEY "pool"

static const char *TAG = "bleprint";

static const ble_uuid16_t UUID_2AF1 = BLE_UUID16_INIT(0x2AF1);
static const ble_uuid128_t UUID_ISSC_WR = BLE_UUID128_INIT(
    0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8,
    0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49);

typedef struct {
    bool              used;
    char              mac[18];
    ble_addr_t        addr;        /* type + 6 byte, hoc duoc tu lan quet dau */
    bool              addr_known;
    volatile uint16_t conn;        /* BLE_HS_CONN_HANDLE_NONE khi rot */
    uint16_t          wr_handle;
    int               wr_rank;     /* 3=2AF1  2=ISSC  1=WRITE bat ky */
    volatile bool     ready;       /* da ket noi + da biet cho ghi */
    int64_t           last_used_us;
} printer_t;

/* ban ghi gon de luu NVS (dia chi hoc duoc song sot qua reboot) */
typedef struct {
    char    mac[18];
    uint8_t addr_type;
    uint8_t addr_val[6];
    uint8_t addr_known;
} printer_nvs_t;

static printer_t         s_prn[MAX_PRINTERS];
static SemaphoreHandle_t s_lock;   /* uplink task vs task nen */
static SemaphoreHandle_t s_step;   /* callbacks -> nguoi cho setup */
static printer_t        *s_setup;  /* entry dang lam quen (giu s_lock) */
static volatile bool     s_failed;
static uint8_t           s_own_addr_type;

/* ------------------------------------------------------------------ NVS */

static void pool_save_nvs(void)
{
    printer_nvs_t rec[MAX_PRINTERS];
    memset(rec, 0, sizeof(rec));
    for (int i = 0; i < MAX_PRINTERS; i++) {
        if (!s_prn[i].used) {
            continue;
        }
        strlcpy(rec[i].mac, s_prn[i].mac, sizeof(rec[i].mac));
        rec[i].addr_type  = s_prn[i].addr.type;
        memcpy(rec[i].addr_val, s_prn[i].addr.val, 6);
        rec[i].addr_known = s_prn[i].addr_known ? 1 : 0;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY, rec, sizeof(rec));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void pool_load_nvs(void)
{
    printer_nvs_t rec[MAX_PRINTERS];
    size_t len = sizeof(rec);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    esp_err_t err = nvs_get_blob(h, NVS_KEY, rec, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(rec)) {
        return;
    }
    for (int i = 0; i < MAX_PRINTERS; i++) {
        if (rec[i].mac[0] == '\0') {
            continue;
        }
        printer_t *p = &s_prn[i];
        p->used = true;
        strlcpy(p->mac, rec[i].mac, sizeof(p->mac));
        p->addr.type = rec[i].addr_type;
        memcpy(p->addr.val, rec[i].addr_val, 6);
        p->addr_known = rec[i].addr_known != 0;
        p->conn = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGI(TAG, "nho may in %s tu NVS%s", p->mac,
                 p->addr_known ? "" : " (chua co dia chi)");
    }
}

/* ------------------------------------------------------------- pool ops */

static void addr_to_str(const ble_addr_t *a, char *out /*>=18*/)
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             a->val[5], a->val[4], a->val[3],
             a->val[2], a->val[1], a->val[0]);
}

static printer_t *pool_by_conn(uint16_t conn)
{
    for (int i = 0; i < MAX_PRINTERS; i++) {
        if (s_prn[i].used && s_prn[i].conn == conn) {
            return &s_prn[i];
        }
    }
    return NULL;
}

/* tim theo MAC; chua co thi lay o trong / thay o LRU (giu s_lock) */
static printer_t *pool_find_or_add(const char *mac)
{
    printer_t *victim = NULL;
    for (int i = 0; i < MAX_PRINTERS; i++) {
        printer_t *p = &s_prn[i];
        if (p->used && strcasecmp(p->mac, mac) == 0) {
            return p;
        }
        if (!p->used) {
            if (victim == NULL || victim->used) {
                victim = p;
            }
        } else if (victim == NULL ||
                   (victim->used && p->last_used_us < victim->last_used_us)) {
            victim = p;
        }
    }
    if (victim->used) { /* day pool: duoi con lau khong dung nhat */
        ESP_LOGW(TAG, "pool day — thay %s bang %s", victim->mac, mac);
        if (victim->conn != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(victim->conn, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    memset(victim, 0, sizeof(*victim));
    victim->used = true;
    strlcpy(victim->mac, mac, sizeof(victim->mac));
    for (char *c = victim->mac; *c != '\0'; c++) {
        if (*c >= 'a' && *c <= 'z') {
            *c -= 32;
        }
    }
    victim->conn = BLE_HS_CONN_HANDLE_NONE;
    pool_save_nvs();
    return victim;
}

/* ------------------------------------------------------ setup callbacks */

static void step_done(bool failed)
{
    if (failed) {
        s_failed = true;
    }
    if (s_step != NULL) {
        xSemaphoreGive(s_step);
    }
}

static int chr_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    printer_t *p = (printer_t *)arg;
    if (error->status == BLE_HS_EDONE) {
        if (p->wr_handle != 0) {
            p->ready = true;
            ESP_LOGI(TAG, "%s san sang (chr ghi val=%u, rank=%d)", p->mac,
                     (unsigned)p->wr_handle, p->wr_rank);
            step_done(false);
        } else {
            ESP_LOGW(TAG, "%s khong co characteristic WRITE nao", p->mac);
            ble_gap_terminate(p->conn, BLE_ERR_REM_USER_CONN_TERM);
            step_done(true);
        }
        return 0;
    }
    if (error->status != 0 || chr == NULL) {
        return 0;
    }
    if (!(chr->properties &
          (BLE_GATT_CHR_PROP_WRITE | BLE_GATT_CHR_PROP_WRITE_NO_RSP))) {
        return 0;
    }
    int rank = 1;
    if (ble_uuid_cmp(&chr->uuid.u, &UUID_2AF1.u) == 0) {
        rank = 3;
    } else if (ble_uuid_cmp(&chr->uuid.u, &UUID_ISSC_WR.u) == 0) {
        rank = 2;
    }
    if (rank > p->wr_rank) {
        p->wr_rank   = rank;
        p->wr_handle = chr->val_handle;
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        printer_t *p = s_setup;
        if (p == NULL) {
            return 0;
        }
        char abuf[18];
        addr_to_str(&event->disc.addr, abuf);
        if (strcasecmp(abuf, p->mac) != 0) {
            return 0;
        }
        /* hoc dia chi (type public/random) — tu nay ket noi thang */
        p->addr       = event->disc.addr;
        p->addr_known = true;
        ESP_LOGI(TAG, "thay may in %s — ket noi...", abuf);
        ble_gap_disc_cancel();
        int rc = ble_gap_connect(s_own_addr_type, &p->addr, CONNECT_MS,
                                 NULL, gap_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "connect rc=%d", rc);
            step_done(true);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_setup != NULL && s_setup->conn == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGW(TAG, "khong thay %s (may in tat? ngoai tam?)",
                     s_setup->mac);
            step_done(true);
        }
        return 0;
    case BLE_GAP_EVENT_CONNECT:
        if (s_setup == NULL) {
            return 0;
        }
        if (event->connect.status == 0) {
            s_setup->conn      = event->connect.conn_handle;
            s_setup->wr_handle = 0;
            s_setup->wr_rank   = 0;
            ble_gattc_disc_all_chrs(s_setup->conn, 1, 0xffff,
                                    chr_disc_cb, s_setup);
        } else {
            ESP_LOGW(TAG, "%s: ket noi loi status=%d", s_setup->mac,
                     event->connect.status);
            step_done(true);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT: {
        printer_t *p = pool_by_conn(event->disconnect.conn.conn_handle);
        if (p != NULL) {
            ESP_LOGW(TAG, "%s rot ket noi (reason=%d) — task nen se noi lai",
                     p->mac, event->disconnect.reason);
            p->conn  = BLE_HS_CONN_HANDLE_NONE;
            p->ready = false;
        }
        if (p != NULL && p == s_setup) {
            step_done(true);
        }
        return 0;
    }
    default:
        return 0;
    }
}

/* lam quen / noi lai mot may in — GIU s_lock khi goi. */
static bool setup_printer(printer_t *p)
{
    while (xSemaphoreTake(s_step, 0) == pdTRUE) { /* xa give cu */ }
    s_setup  = p;
    s_failed = false;
    p->ready = false;
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_gap_disc_cancel(); /* nhuong cho (ble_central co the dang quet) */

    int rc;
    if (p->addr_known) {
        /* da biet dia chi: ket noi thang, khoi quet (nhanh hon nhieu) */
        rc = ble_gap_connect(s_own_addr_type, &p->addr, CONNECT_MS,
                             NULL, gap_event_cb, NULL);
    } else {
        struct ble_gap_disc_params dp = { .passive = 1 };
        rc = ble_gap_disc(s_own_addr_type, SCAN_MS, &dp, gap_event_cb, NULL);
    }
    if (rc != 0) {
        ESP_LOGW(TAG, "%s: start setup rc=%d", p->mac, rc);
        s_setup = NULL;
        return false;
    }

    bool signalled = false;
    for (int waited = 0; waited < SETUP_TIMEOUT_MS; waited += 500) {
        esp_task_wdt_reset();
        if (xSemaphoreTake(s_step, pdMS_TO_TICKS(500)) == pdTRUE) {
            signalled = true;
            break;
        }
    }
    bool ok = signalled && !s_failed && p->ready &&
              p->conn != BLE_HS_CONN_HANDLE_NONE;
    if (!ok) {
        ble_gap_disc_cancel();
        if (p->conn != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(p->conn, BLE_ERR_REM_USER_CONN_TERM);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    } else if (p->addr_known) {
        pool_save_nvs(); /* dia chi vua hoc duoc song sot qua reboot */
    }
    s_setup = NULL;
    return ok;
}

/* ------------------------------------------------------------- ghi data */

static bool write_all(printer_t *p, const uint8_t *data, size_t len)
{
    uint16_t mtu   = ble_att_mtu(p->conn);
    size_t   chunk = (size_t)(mtu > 23 ? mtu : 23) - 3;
    size_t   off = 0, sent_since_pause = 0;
    int      retries = 0;

    while (off < len) {
        if (p->conn == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGW(TAG, "%s: mat ket noi giua chung (%u/%u bytes)",
                     p->mac, (unsigned)off, (unsigned)len);
            return false;
        }
        size_t n = len - off < chunk ? len - off : chunk;
        int rc = ble_gattc_write_no_rsp_flat(p->conn, p->wr_handle,
                                             data + off, n);
        if (rc == BLE_HS_ENOMEM) {
            if (++retries > CHUNK_RETRY_MAX) {
                ESP_LOGW(TAG, "%s: buffer BLE nghen qua lau", p->mac);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            esp_task_wdt_reset();
            continue;
        }
        if (rc != 0) {
            ESP_LOGW(TAG, "%s: write rc=%d tai offset %u", p->mac, rc,
                     (unsigned)off);
            return false;
        }
        retries = 0;
        off += n;
        sent_since_pause += n;
        if (sent_since_pause >= 512) {
            /* may in nhiet an data cham hon BLE day: tha nhip nhe */
            vTaskDelay(pdMS_TO_TICKS(20));
            esp_task_wdt_reset();
            sent_since_pause = 0;
        }
    }
    return true;
}

/* --------------------------------------------------------------- public */

static void take_lock(void)
{
    while (xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
        esp_task_wdt_reset(); /* uplink task nam trong task wdt */
    }
}

esp_err_t ble_print_run(const char *mac, const uint8_t *data, size_t len)
{
    if (mac == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE; /* ble_print_init chua chay */
    }
    if (!ble_hs_synced()) {
        ESP_LOGW(TAG, "BLE host chua san sang");
        return ESP_ERR_INVALID_STATE;
    }

    take_lock();
    printer_t *p = pool_find_or_add(mac);
    p->last_used_us = esp_timer_get_time();

    bool ok = false;
    if (p->ready || setup_printer(p)) {
        ok = write_all(p, data, len);
        if (!ok && p->conn == BLE_HS_CONN_HANDLE_NONE) {
            /* rot dung luc in: lam quen lai mot lan roi in lai tu dau */
            if (setup_printer(p)) {
                ok = write_all(p, data, len);
            }
        }
    }
    /* GIU ket noi (khong terminate) — lan in sau chi ton ~0.3s */
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "%s: in %s (%u bytes)", mac, ok ? "XONG" : "LOI",
             (unsigned)len);
    return ok ? ESP_OK : ESP_FAIL;
}

/* KHONG co task nen tu noi lai (bo 20/07 theo quyet dinh user — don gian
 * hon): ket noi thanh cong thi GIU; rot thi JOB KE TIEP tu noi lai ngay
 * trong ble_print_run (da biet dia chi tu NVS nen noi thang, khong quet)
 * — tem dau tien sau su co/reboot cham hon ~2-3s, fail thi bam Retry. */
esp_err_t ble_print_init(void)
{
    if (s_lock != NULL) {
        return ESP_OK;
    }
    s_lock = xSemaphoreCreateMutex();
    s_step = xSemaphoreCreateBinary();
    if (s_lock == NULL || s_step == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < MAX_PRINTERS; i++) {
        s_prn[i].conn = BLE_HS_CONN_HANDLE_NONE;
    }
    pool_load_nvs(); /* dia chi da hoc: job dau sau reboot noi thang */
    return ESP_OK;
}
