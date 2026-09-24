#include "sdkconfig.h"

#include "uplink.h"

/* Tat CONFIG_UPLINK_HTTP_ENABLE thi ca file nay thanh mot bo stub.
 *
 * Phai bao het ca file chu khong chi rieng uplink_start(): moi thu ben
 * duoi — esp_http_client, bo dem 4 KB, ngan xep 8 KB cua uplink_task —
 * deu chi ton tai vi duong HTTP. Bao ca file de trinh lien ket vut chung
 * di that, chu khong chi de chung nam yen.
 *
 * Bon ham con lai van phai tra loi dung, vi ben goi khong biet gi ve viec
 * nay:
 *   uplink_server_ok()  -> tower_light lay lam den XANH bao suc khoe. Tra
 *                          false cung deu thi den tat vinh vien, nhin nhu
 *                          hong. Voi HTTP tat, "con noi duoc voi may chu"
 *                          nghia la con noi duoc BROKER.
 *   uplink_sent_total() -> main.c in ra moi 10 giay; lay so cua MQTT.
 *   uplink_kick()       -> scale_serial goi khi co lan can moi, de ban ghi
 *                          nghiep vu di ngay thay vi doi het chu ky.
 */
#if !CONFIG_UPLINK_HTTP_ENABLE

#include "mqtt_link.h"

esp_err_t uplink_start(void)      { return ESP_OK; }
uint32_t  uplink_sent_total(void) { return mqtt_link_published(); }
bool      uplink_server_ok(void)  { return mqtt_link_connected(); }
void      uplink_kick(void)       { mqtt_link_kick(); }
void      uplink_set_tower_cb(uplink_tower_cb_t cb) { (void)cb; }

#else

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "mbedtls/base64.h"

#include "ble_print.h"
#include "cfg.h"
#include "gpio_out.h"
#include "meas_core.h"
#include "net_mgr.h"
#include "spooler.h"
#include "uplink.h"

/* edge_collector contract (../edge_collector/edge_collector/node_api.py);
 * node_agent (../node_agent/node_agent/edge_client.py) is the reference
 * client this firmware mirrors. */
#define PATH_HELLO          "/node/v1/hello"
#define PATH_MEASUREMENTS   "/node/v1/measurements"
#define PATH_HEARTBEAT      "/node/v1/heartbeat"
#define PATH_COMMANDS       "/node/v1/commands"
#define PATH_COMMANDS_ACK   "/node/v1/commands/ack"
#define PATH_CONFIG         "/node/v1/config"

#define BATCH_MAX        50     /* per-upload cap; nho de dinh RAM khi xa backlog */
#define HTTP_TIMEOUT_MS  10000
#define BACKOFF_MIN_MS   2000
/* Tran backoff 30s: edge trong LAN, ket noi lai re — sau khi edge bat lai
 * node thu lai trong <=30s (khong cho 5 phut nhu remote link). */
#define BACKOFF_MAX_MS   30000
#define RESP_BUF_SIZE    512
/* Bo dem TINH dung goi measurements. Mot ban ghi ~110 byte -> 4 KB chua
 * ~35 ban ghi; day thi cat lo, phan con lai di lo sau. Tinh chu khong
 * malloc de khong phan manh heap tren duong nong. */
#define BODY_BYTES       4096
#define CFG_JSON_MAX     16384           /* matches cfg.c CHJSON_MAX */
#define CFG_RETRY_US     (5LL * 60 * 1000000)  /* min gap between fetches */
#define CFG_DRAIN_BATCHES 20             /* spool flush cap before restart */
#define HELLO_INTERVAL_S    60  /* matches node_agent NODE_HELLO_INTERVAL_S */
#define CMD_POLL_INTERVAL_S 2   /* matches node_agent NODE_COMMAND_POLL_INTERVAL_S */
/* timestamps below this are pre-SNTP (device booted in 1970) and get
 * repaired with net_mgr_time_offset_ms(); 2020-01-01 UTC */
#define EPOCH_SANE_MS    1577836800000LL

static const char *TAG = "uplink";

static measurement_t s_batch[BATCH_MAX];
static char          s_resp[RESP_BUF_SIZE];
static int           s_resp_len;
static uint32_t      s_sent_total;
static uint32_t      s_backoff_ms;
static int64_t       s_last_ok_us;
static uplink_tower_cb_t s_tower_cb;
static uint32_t      s_server_cfg_ver;   /* last config_version seen in a
                                            server response; 0 = none yet */
static int64_t       s_cfg_fail_us;      /* last failed /config fetch */
static volatile bool s_print_pending;    /* server co lenh in cho minh */

/* co "print_pending" trong response (heartbeat + ack do): server dang giu
 * lenh in cho node nay -> task loop se GET /print_jobs/next va in.
 * edge_collector's /node/v1/... replies never carry this key today (no
 * print_jobs endpoint exists there yet, see node_api.py) — kept unused
 * (__attribute__((unused)) to silence -Wunused-function) so a future
 * contract addition is a one-line call-site change instead of a rewrite. */
static void __attribute__((unused)) note_print_flag(const cJSON *root)
{
    const cJSON *p = cJSON_GetObjectItemCaseSensitive(root, "print_pending");
    if (cJSON_IsBool(p)) {
        s_print_pending = cJSON_IsTrue(p);
    }
}

/* remember the server's config_version; acted on in check_config_update()
 * (called from the task loop, never from inside a response handler) */
static void note_server_version(const cJSON *root)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "config_version");
    if (cJSON_IsNumber(v) && v->valuedouble >= 0) {
        s_server_cfg_ver = (uint32_t)v->valuedouble;
    }
}

/* forward the optional {"tower":{...}} of a parsed response to tower_light.
 * Same situation as note_print_flag() above: edge_collector's /node/v1/...
 * replies never carry a "tower" key today, so this is currently unreachable
 * (kept + marked unused rather than deleted, along with uplink_set_tower_cb
 * in uplink.h — tower_light's OWN local alarm-threshold logic is unaffected
 * and keeps driving the lamp regardless). */
static void __attribute__((unused)) handle_tower(const cJSON *root)
{
    const cJSON *t = cJSON_GetObjectItemCaseSensitive(root, "tower");
    if (!cJSON_IsObject(t) || s_tower_cb == NULL) {
        return;
    }
    const char *keys[5] = { "r", "y", "g", "b", "bz" };
    int v[5];
    for (int i = 0; i < 5; i++) {
        const cJSON *it = cJSON_GetObjectItemCaseSensitive(t, keys[i]);
        v[i] = cJSON_IsNumber(it) ? (it->valuedouble != 0) : -1;
    }
    s_tower_cb(v[0], v[1], v[2], v[3], v[4]);
}

/* long sleeps must keep petting the task watchdog (30 s timeout) */
static void sleep_ms_wdt(uint32_t ms)
{
    while (ms > 0) {
        uint32_t step = ms > 1000 ? 1000 : ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        esp_task_wdt_reset();
        ms -= step;
    }
}

static esp_err_t http_evt(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        int space = RESP_BUF_SIZE - 1 - s_resp_len;
        int n = evt->data_len < space ? evt->data_len : space;
        if (n > 0) {
            memcpy(s_resp + s_resp_len, evt->data, n);
            s_resp_len += n;
        }
    }
    return ESP_OK;
}

/* X-Device-Serial luon can; X-API-Key CHI gui khi da co gia tri (rong = chua
 * hoc duoc qua /hello) — gui header rong se bi tu choi neu edge da biet mot
 * khoa khac cho serial nay (xem node_api.py:_auth), giong het cach
 * edge_client.py (node_agent) chi them header khi self.api_key co gia tri. */
static void set_auth_headers(esp_http_client_handle_t cl, bool include_key)
{
    esp_http_client_set_header(cl, "X-Device-Serial", cfg_node_serial());
    if (include_key) {
        const char *key = cfg_get_api_key();
        if (key != NULL && key[0] != '\0') {
            esp_http_client_set_header(cl, "X-API-Key", key);
        }
    }
}

/* POST body to path; returns HTTP status (or <0 on transport error) and
 * leaves the response body NUL-terminated in s_resp. include_key=false for
 * /node/v1/hello, the one path that must never send a (possibly stale)
 * X-API-Key — see node_api.py's node_hello() comment. */
/* ── mot handle HTTP dung lai cho moi request ───────────────────────────
 *
 * Truoc day http_post_json()/http_get_buf() deu esp_http_client_init() roi
 * cleanup() MOI LAN GOI. Moi lan init dung lai cau truc client, bo dem
 * thu/phat, tang van chuyen TCP, bo phan tich URL, danh sach header —
 * khoang 15 KB, cap roi tra, ~20 lan moi giay.
 *
 * Do 19/09 bang heapwatch: heap trong 24,5 KB, day cham 9,4 KB, va day cu
 * troi xuong dan vi phan manh. `.keep_alive_enable = true` da bat tu truoc
 * nhung VO NGHIA khi handle bi huy ngay sau do — giu ket noi chi co y
 * nghia trong vong doi cua handle.
 *
 * AN TOAN LUONG: moi nguoi goi (hello, heartbeat, measurements, command,
 * print) deu chay trong uplink_task. Neu sau nay co task khac goi vao day
 * thi PHAI them khoa.
 */
static esp_http_client_handle_t s_http;
static char s_http_base[128];

static void http_drop(void)
{
    if (s_http != NULL) {
        esp_http_client_cleanup(s_http);
        s_http = NULL;
        s_http_base[0] = '\0';
    }
}

/* Tra ve handle da tro san toi `url`, tao moi neu chua co hoac neu
 * server_url da doi (nguoi dung sua qua trang /setup). */
static esp_http_client_handle_t http_client_for(const char *url)
{
    const char *base = cfg_get_server_url();
    if (s_http != NULL && strcmp(s_http_base, base) != 0) {
        ESP_LOGI(TAG, "server_url doi, tao lai ket noi");
        http_drop();
    }
    if (s_http == NULL) {
        esp_http_client_config_t cc = {
            .url           = url,
            .timeout_ms    = HTTP_TIMEOUT_MS,
            .event_handler = http_evt,
            .keep_alive_enable = true,
            /* Gan kho CA CHI KHI url that su la https.
             *
             * Chu thich cu noi bundle "chi duoc dung cho https, bo qua voi
             * http thuan". Dung ve SU DUNG, sai ve CAP PHAT: dat truong nay
             * khien esp_http_client_init() dung ca bo may TLS kem kho chung
             * chi. Do duoc: ~6 KB trong tong ~21,5 KB moi POST. */
            .crt_bundle_attach = (strncmp(url, "https://", 8) == 0)
                                 ? esp_crt_bundle_attach : NULL,
        };
        s_http = esp_http_client_init(&cc);
        if (s_http == NULL) {
            return NULL;
        }
        strlcpy(s_http_base, base, sizeof(s_http_base));
    } else if (esp_http_client_set_url(s_http, url) != ESP_OK) {
        http_drop();
        return NULL;
    }
    return s_http;
}

static int http_post_json(const char *path, const char *body, bool include_key)
{
    char url[192];
    snprintf(url, sizeof(url), "%s%s", cfg_get_server_url(), path);

    s_resp_len = 0;
    s_resp[0]  = '\0';

    esp_http_client_handle_t cl = http_client_for(url);
    if (cl == NULL) {
        return -1;
    }
    esp_http_client_set_method(cl, HTTP_METHOD_POST);
    esp_http_client_set_header(cl, "Content-Type", "application/json");
    set_auth_headers(cl, include_key);
    esp_http_client_set_post_field(cl, body, strlen(body));

    esp_err_t err = esp_http_client_perform(cl);
    int status = (err == ESP_OK) ? esp_http_client_get_status_code(cl) : -1;
    if (err != ESP_OK) {
        /* ket noi hong: bo handle di de lan sau mo lai sach, khong om mot
         * socket da chet */
        http_drop();
    }

    s_resp[s_resp_len] = '\0';
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST %s: %s", path, esp_err_to_name(err));
    }
    return status;
}

/* {"items":[{ch,v,s,q,ts,stable}], "bid", "seq"} theo node_api.py's
 * /node/v1/measurements. "s" (string reading) khong co tuong duong tren
 * ESP32 (moi kenh deu la so) nen luon di ra null, giong node_agent.
 *
 * Dung chuoi bang snprintf vao bo dem TINH, khong dung cay cJSON nua.
 *
 * Vi sao doi: mot cay cJSON cho 50 ban ghi la ~350 nut x ~64 byte = ~22 KB
 * NHAT THOI, cong chuoi in ra, cong bo dem cua cJSON_Print tu nhan doi khi
 * day. Heap trong cua node nay chi 48 KB (PSRAM co chu dich khong bat, xem
 * sdkconfig.defaults). Do duoc 19/09 khi them duong MQTT chay song song —
 * hai cay cJSON cung luc: min_heap tut con 448 byte, http_post_json khong
 * cap phat noi bo dem, `sent` dung o 0 va spool day cung 2048.
 *
 * Tra ve so ban ghi DA VIET DUOC — co the it hon n neu bo dem day. Nguoi
 * goi PHAI ack theo con so nay, khong phai theo n.
 */
static char s_body[BODY_BYTES];

static size_t build_measurements_body(const measurement_t *batch, size_t n,
                                      size_t *out_len)
{
    int w = snprintf(s_body, sizeof(s_body),
                     "{\"bid\":%u,\"seq\":%" PRIu32 ",\"items\":[",
                     (unsigned)batch[n - 1].boot_id, batch[n - 1].seq);
    if (w < 0 || (size_t)w >= sizeof(s_body)) {
        return 0;
    }
    size_t len = (size_t)w;
    size_t done = 0, written = 0;
    int64_t offset = net_mgr_time_offset_ms();

    for (size_t i = 0; i < n; i++) {
        const measurement_t *m = &batch[i];
        const cfg_channel_t *ch = cfg_channel_by_id(m->channel_id);
        if (ch == NULL) {
            /* kenh da bi go khoi bang: bo qua nhung VAN tinh la da xu ly,
             * neu khong no ket lai dau spool mai mai */
            done++;
            continue;
        }
        int64_t ts = m->ts_ms;
        if (ts < EPOCH_SANE_MS) {
            ts += offset; /* recorded before first SNTP sync */
        }
        int k2 = snprintf(s_body + len, sizeof(s_body) - len,
                          "%s{\"ch\":\"%s\",\"v\":%.4f,\"s\":null,\"q\":%u,"
                          "\"ts\":%lld,\"stable\":%s}",
                          written ? "," : "", ch->code,
                          (double)m->value, (unsigned)m->quality,
                          (long long)ts,
                          m->quality != Q_UNSTABLE ? "true" : "false");
        if (k2 < 0 || (size_t)k2 >= sizeof(s_body) - len) {
            break;      /* day bo dem: gui phan da co, phan con lai o lai
                         * spool va di trong lo sau */
        }
        len += (size_t)k2;
        written++;
        done++;
    }
    if (done == 0) {
        return 0;
    }
    int k3 = snprintf(s_body + len, sizeof(s_body) - len, "]}");
    if (k3 < 0 || (size_t)k3 >= sizeof(s_body) - len) {
        return 0;
    }
    *out_len = len + (size_t)k3;
    return done;
}


/* returns true when the batch was consumed (acked or deliberately dropped) */
static bool upload_batch(const measurement_t *batch, size_t n)
{
    size_t body_len = 0;
    size_t done = build_measurements_body(batch, n, &body_len);
    if (done == 0) {
        return false;
    }
    int status = http_post_json(PATH_MEASUREMENTS, s_body, true);

    /* Ack theo `done`, KHONG theo n: bo dem tinh co the chi chua duoc mot
     * phan lo. Ack qua ca lo se xoa nhung ban ghi chua he roi khoi node. */
    uint16_t last_bid = batch[done - 1].boot_id;
    uint32_t last_seq = batch[done - 1].seq;

    if (status == 200) {
        /* node_measurements() replies {"ok":true,"accepted":n} only — no
         * per-record ack, no tower/config_version/print_pending (those are
         * heartbeat-only under this contract). A 200 means the whole batch
         * was accepted, so ack through the batch's own last record. */
        spool_ack_through(last_bid, last_seq);
        __atomic_add_fetch(&s_sent_total, (uint32_t)done, __ATOMIC_RELAXED);
        s_last_ok_us = esp_timer_get_time();
        return true;
    }
    if (status == 400) {
        /* malformed by server's judgement: never retry-loop a poison batch */
        ESP_LOGE(TAG, "server rejected batch (400), dropping %u samples: %s",
                 (unsigned)done, s_resp);
        spool_ack_through(last_bid, last_seq);
        return true;
    }
    if (status == 401 || status == 403) {
        ESP_LOGE(TAG, "auth rejected (%d), retry in 60 s", status);
        sleep_ms_wdt(60000);
        return false;
    }
    if (status == 413 && n > 1) {
        /* too large: split — upload halves recursively */
        ESP_LOGW(TAG, "413, splitting batch of %u", (unsigned)n);
        return upload_batch(batch, n / 2) && upload_batch(batch + n / 2, n - n / 2);
    }
    return false; /* 5xx / timeout: caller backs off, spool keeps the data */
}

/* node_heartbeat() forwards this body VERBATIM to Odoo's own heartbeat
 * endpoint (agent.forward_node_heartbeat -> odoo.heartbeat), so extra fields
 * beyond node_agent's minimal {fw,config_version} are harmless — but the
 * reply back to us is always exactly {"ok":..,"config_version":..}: no
 * tower/print_pending/server_time_ms any more (time now comes from hello). */
static void send_heartbeat(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "fw", "0.1.0");
    if (cfg_loc_name()[0] != '\0') {
        /* server uses this to name the device on first contact only */
        cJSON_AddStringToObject(root, "name", cfg_loc_name());
    }
    cJSON_AddNumberToObject(root, "uptime_s",
                            (double)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(root, "rssi", net_mgr_rssi());
    if (net_mgr_ip()[0] != '\0') {
        /* dashboard turns this into a link to the node's config page */
        cJSON_AddStringToObject(root, "ip", net_mgr_ip());
    }
    cJSON_AddNumberToObject(root, "free_heap", (double)esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "buffered", (double)spool_depth());
    cJSON_AddNumberToObject(root, "config_version", cfg_config_version());
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body != NULL) {
    int status = http_post_json(PATH_HEARTBEAT, body, true);
        cJSON_free(body);
        if (status == 200) {
            s_last_ok_us = esp_timer_get_time();
            cJSON *r = cJSON_Parse(s_resp);
            if (r != NULL) {
                note_server_version(r);
                cJSON_Delete(r);
            }
        } else {
            ESP_LOGW(TAG, "heartbeat status %d", status);
        }
    }
}

/* POST /node/v1/hello {name,kind} -> {ok,known,api_key,server_time_ms}.
 * KHONG gui X-API-Key (include_key=false): day la duong DUY NHAT de hoc/hoc
 * lai khoa — gui kem mot khoa cu/sai se bi tu choi va khoa dung (Odoo da
 * cap) khong bao gio hoc duoc nua, dung y het ly do trong edge_client.py.
 * Cung la nguon server_time_ms cho net_mgr khi chua co SNTP, thay cho
 * heartbeat (response cua no khong con mang truong nay nua). */
static void send_hello(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", cfg_loc_name());
    cJSON_AddStringToObject(root, "kind", "other"); /* pcm.device.kind: pi|pc|other */
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return;
    }
    int status = http_post_json(PATH_HELLO, body, false);
    cJSON_free(body);
    if (status != 200) {
        ESP_LOGI(TAG, "hello status %d", status);
        return;
    }
    cJSON *r = cJSON_Parse(s_resp);
    if (r == NULL) {
        return;
    }
    const cJSON *key = cJSON_GetObjectItemCaseSensitive(r, "api_key");
    if (cJSON_IsString(key) && key->valuestring[0] != '\0' &&
        strcmp(key->valuestring, cfg_get_api_key()) != 0) {
        cfg_set_api_key(key->valuestring);
        ESP_LOGI(TAG, "da hoc api_key moi tu edge (/hello)");
    }
    const cJSON *st = cJSON_GetObjectItemCaseSensitive(r, "server_time_ms");
    if (cJSON_IsNumber(st)) {
        /* no-op once NET_BIT_TIME is set (SNTP stays authoritative) */
        net_mgr_time_from_server((int64_t)st->valuedouble);
    }
    cJSON_Delete(r);
}

/* GET path into caller's buffer (open/read, so s_resp stays untouched).
 * Returns body length, or -1 on transport error / oversized body; HTTP
 * status goes to *out_status. */
static int http_get_buf(const char *path, char *buf, int cap, int *out_status)
{
    char url[192];
    snprintf(url, sizeof(url), "%s%s", cfg_get_server_url(), path);

    esp_http_client_handle_t cl = http_client_for(url);
    if (cl == NULL) {
        return -1;
    }
    esp_http_client_set_method(cl, HTTP_METHOD_GET);
    /* Xoa than goi cua lan POST truoc — handle dung chung nen no con dinh
     * lai, va GET keo theo than la loi. */
    esp_http_client_set_post_field(cl, NULL, 0);
    set_auth_headers(cl, true);

    int len = -1;
    *out_status = -1;
    if (esp_http_client_open(cl, 0) == ESP_OK) {
        esp_http_client_fetch_headers(cl);
        *out_status = esp_http_client_get_status_code(cl);
        len = 0;
        while (len < cap) {
            int n = esp_http_client_read(cl, buf + len, cap - len);
            if (n < 0) {
                len = -1;
                break;
            }
            if (n == 0) {
                break;
            }
            len += n;
        }
        if (len == cap && !esp_http_client_is_complete_data_received(cl)) {
            ESP_LOGE(TAG, "GET %s: body larger than %d, dropped", path, cap);
            len = -1;
        }
    }
    esp_http_client_close(cl);
    if (len < 0) {
        http_drop();    /* doc hong: mo lai sach o lan sau */
    }
    return len;
}

/* Server announced a config_version different from ours (seen in the
 * heartbeat response): download /config, validate + persist it via cfg,
 * flush what the spool holds and restart to apply. Failures back off
 * CFG_RETRY_US so a broken server can't cause a loop.
 *
 * NOT CALLED right now (see the call site's comment in uplink_task): under
 * edge_collector/pcm_base this "config_version" is edge-wide, not per-node
 * wiring, and /node/v1/config carries no bus/host/reg — applying it here
 * silently destroyed real channel config. Kept + marked unused rather than
 * deleted so it can be re-enabled once pcm.channel actually carries wiring
 * fields the same way the old iot.channel does. */
static void __attribute__((unused)) check_config_update(void)
{
    uint32_t want = s_server_cfg_ver;
    if (want == 0 || want == cfg_config_version()) {
        return;
    }
    if (s_cfg_fail_us != 0 &&
        esp_timer_get_time() - s_cfg_fail_us < CFG_RETRY_US) {
        return;
    }
    ESP_LOGI(TAG, "config v%" PRIu32 " available (running v%" PRIu32
             "), fetching /config", want, cfg_config_version());

    char *buf = malloc(CFG_JSON_MAX);
    if (buf == NULL) {
        return;
    }
    int status = -1;
    int len = http_get_buf(PATH_CONFIG, buf, CFG_JSON_MAX - 1, &status);
    if (len <= 0 || status != 200) {
        ESP_LOGW(TAG, "config fetch failed (status %d)", status);
        s_cfg_fail_us = esp_timer_get_time();
        free(buf);
        return;
    }
    buf[len] = '\0';

    /* node_config() replies {"ok":true,"channels":[...]} — no config_version
     * at the top level like cfg_store_channels_json() requires (that shape
     * matches Odoo's own /pcm/api/v1/edge/config, not this node contract).
     * Splice in the version we already know from the heartbeat ("want")
     * before handing the document to cfg. */
    cJSON *resp = cJSON_Parse(buf);
    free(buf);
    if (resp == NULL) {
        ESP_LOGW(TAG, "config fetch: invalid JSON");
        s_cfg_fail_us = esp_timer_get_time();
        return;
    }
    cJSON *channels = cJSON_DetachItemFromObjectCaseSensitive(resp, "channels");
    cJSON_Delete(resp);
    if (!cJSON_IsArray(channels)) {
        ESP_LOGW(TAG, "config fetch: no channels array");
        cJSON_Delete(channels);
        s_cfg_fail_us = esp_timer_get_time();
        return;
    }
    cJSON *doc = cJSON_CreateObject();
    cJSON_AddNumberToObject(doc, "config_version", want);
    cJSON_AddItemToObject(doc, "channels", channels); /* doc now owns channels */
    char *doc_str = cJSON_PrintUnformatted(doc);
    cJSON_Delete(doc);
    if (doc_str == NULL) {
        s_cfg_fail_us = esp_timer_get_time();
        return;
    }

    esp_err_t store_err = cfg_store_channels_json(doc_str, strlen(doc_str));
    cJSON_free(doc_str);
    if (store_err != ESP_OK) {
        /* invalid document: back off, keep running the current table */
        s_cfg_fail_us = esp_timer_get_time();
        return;
    }

    /* RAM spool does not survive the reboot: best-effort flush first */
    for (int i = 0; i < CFG_DRAIN_BATCHES; i++) {
        size_t n = spool_peek(s_batch, BATCH_MAX);
        if (n == 0 || !upload_batch(s_batch, n)) {
            break;
        }
    }
    ESP_LOGI(TAG, "restarting to apply config v%" PRIu32, want);
    esp_restart();
}

/* Thuc thi mot lenh da poll duoc. Hien tai CHI co executor cho kenh GPIO
 * output (relay/den — bus="gpio", mode="output" trong channels.json, xem
 * gpio_out.c): cmd="write" ghi muc 0/1 that ra chan do. Moi kenh/lenh khac
 * (measure/serial/mbrtu/... hoac cmd != write) van bi tu choi co chu dich —
 * chua co executor, khong gia vo thanh cong. */
/* GET /node/v1/commands -> {"command":{"id","channel","cmd","value"}} |
 * {"command":null}. Ket qua thuc thi (xem execute_command() o tren) duoc
 * ACK ve /node/v1/commands/ack de tra loi lai cho Odoo's
 * SourceManager.queue_command() (dang cho, 8 s timeout) ngay lap tuc thay
 * vi de no tu het gio. */
static void poll_command(void)
{
    char buf[256];
    int status = -1;
    int len = http_get_buf(PATH_COMMANDS, buf, sizeof(buf) - 1, &status);
    if (len <= 0 || status != 200) {
        return;
    }
    buf[len] = '\0';
    cJSON *r = cJSON_Parse(buf);
    if (r == NULL) {
        return;
    }
    const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(r, "command");
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(cmd, "id");
    if (cJSON_IsObject(cmd) && cJSON_IsNumber(id)) {
        const cJSON *ch    = cJSON_GetObjectItemCaseSensitive(cmd, "channel");
        const cJSON *op    = cJSON_GetObjectItemCaseSensitive(cmd, "cmd");
        const cJSON *value = cJSON_GetObjectItemCaseSensitive(cmd, "value");
        const char *ch_code = cJSON_IsString(ch) ? ch->valuestring : "";
        const char *op_str  = cJSON_IsString(op) ? op->valuestring : "";

        const cJSON *jms  = cJSON_GetObjectItemCaseSensitive(cmd, "ms");
        const cJSON *jper = cJSON_GetObjectItemCaseSensitive(cmd, "period_ms");
        const gpio_cmd_t gc = {
            .channel   = ch_code,
            .op        = op_str,
            .has_value = cJSON_IsNumber(value),
            .value     = cJSON_IsNumber(value) ? value->valuedouble : 0.0,
            .ms        = cJSON_IsNumber(jms)  ? (int32_t)jms->valuedouble  : 0,
            .period_ms = cJSON_IsNumber(jper) ? (int32_t)jper->valuedouble : 0,
        };
        char detail[64];
        bool ok = gpio_out_execute(&gc, detail, sizeof(detail));
        if (ok) {
            ESP_LOGI(TAG, "lenh '%s' tren kenh '%s' thuc thi OK", op_str, ch_code);
        } else {
            ESP_LOGW(TAG, "lenh '%s' tren kenh '%s' bi tu choi: %s",
                     op_str, ch_code, detail);
        }

        cJSON *ack = cJSON_CreateObject();
        cJSON_AddNumberToObject(ack, "id", id->valuedouble);
        cJSON_AddBoolToObject(ack, "ok", ok);
        if (!ok) {
            cJSON_AddStringToObject(ack, "detail", detail);
        }
        char *ack_body = cJSON_PrintUnformatted(ack);
        cJSON_Delete(ack);
        if (ack_body != NULL) {
            http_post_json(PATH_COMMANDS_ACK, ack_body, true);
            cJSON_free(ack_body);
        }
    }
    cJSON_Delete(r);
}

/* -------------------------------------------------- lenh in (BLE printer) */
#define PRINT_JSON_MAX 24576  /* job JSON (payload base64) toi da ~24KB */
#define PRINT_JOBS_PER_PASS 3 /* chan thoi gian chiem task moi vong */

/* Old ingest contract's print-job queue (/api/iot/v1/print_jobs/...) — kept
 * for the same reason as note_print_flag()/handle_tower() above:
 * edge_collector has no equivalent endpoint yet (no print_pending flag is
 * ever set, see note_print_flag()), so run_print_jobs() below always
 * no-ops. Left in place rather than deleted so wiring a future
 * edge_collector print queue back in is a small change, not a rewrite.
 *
 * GET /print_jobs/next -> {"job":{"id","mac","payload_b64"}} | {"job":null}
 * -> giai base64 -> ble_print_run -> POST /print_jobs/ack. Chay dong bo
 * trong uplink task (in mat 5-30s; do la chap nhan duoc, spool van gom
 * mau trong luc do). Tra true khi CON job dang cho (goi lai vong sau). */
static bool run_one_print_job(void)
{
    char *buf = malloc(PRINT_JSON_MAX);
    if (buf == NULL) {
        return false;
    }
    int status = -1;
    int len = http_get_buf("/api/iot/v1/print_jobs/next", buf,
                           PRINT_JSON_MAX - 1, &status);
    if (len <= 0 || status != 200) {
        free(buf);
        return false;
    }
    buf[len] = '\0';
    cJSON *r = cJSON_Parse(buf);
    free(buf);
    if (r == NULL) {
        return false;
    }
    const cJSON *job = cJSON_GetObjectItemCaseSensitive(r, "job");
    const cJSON *jid = cJSON_GetObjectItemCaseSensitive(job, "id");
    const cJSON *jmac = cJSON_GetObjectItemCaseSensitive(job, "mac");
    const cJSON *jb64 = cJSON_GetObjectItemCaseSensitive(job, "payload_b64");
    if (!cJSON_IsObject(job) || !cJSON_IsNumber(jid) ||
        !cJSON_IsString(jmac) || !cJSON_IsString(jb64)) {
        cJSON_Delete(r);
        return false; /* het job (job:null) hoac JSON la */
    }

    int    id  = (int)jid->valuedouble;
    char   mac[18];
    strlcpy(mac, jmac->valuestring, sizeof(mac));

    size_t b64len = strlen(jb64->valuestring);
    size_t cap    = b64len / 4 * 3 + 4;
    uint8_t *raw  = malloc(cap);
    size_t olen   = 0;
    esp_err_t res = ESP_FAIL;
    const char *detail = "";
    if (raw == NULL) {
        detail = "no mem";
    } else if (mbedtls_base64_decode(raw, cap, &olen,
                                     (const uint8_t *)jb64->valuestring,
                                     b64len) != 0 || olen == 0) {
        detail = "base64 loi";
    } else {
        ESP_LOGI(TAG, "lenh in #%d: %u bytes -> %s", id, (unsigned)olen, mac);
        res = ble_print_run(mac, raw, olen);
        detail = (res == ESP_OK) ? "" : "khong ghi duoc qua BLE"
                                        " (may in tat/ngoai tam?)";
    }
    free(raw);
    cJSON_Delete(r);

    cJSON *ack = cJSON_CreateObject();
    cJSON_AddNumberToObject(ack, "id", id);
    cJSON_AddBoolToObject(ack, "ok", res == ESP_OK);
    if (detail[0] != '\0') {
        cJSON_AddStringToObject(ack, "detail", detail);
    }
    char *body = cJSON_PrintUnformatted(ack);
    cJSON_Delete(ack);
    if (body != NULL) {
        http_post_json("/api/iot/v1/print_jobs/ack", body, true);
        cJSON_free(body);
    }
    /* in loi: dung vong (job giu trang thai loi tren Odoo, nguoi van hanh
     * bam In lai); in ok: thu tiep bien dau con job xep hang */
    return res == ESP_OK;
}

static void run_print_jobs(void)
{
    if (!s_print_pending) {
        return;
    }
    s_print_pending = false; /* set lai boi response ke tiep neu con */
    for (int i = 0; i < PRINT_JOBS_PER_PASS; i++) {
        esp_task_wdt_reset();
        if (!run_one_print_job()) {
            break;
        }
    }
}

static void backoff_sleep(void)
{
    if (s_backoff_ms == 0) {
        s_backoff_ms = BACKOFF_MIN_MS;
    } else {
        s_backoff_ms *= 2;
        if (s_backoff_ms > BACKOFF_MAX_MS) {
            s_backoff_ms = BACKOFF_MAX_MS;
        }
    }
    uint32_t jitter = s_backoff_ms / 100 * (esp_random() % 21); /* 0-20% */
    ESP_LOGW(TAG, "upload failed, backoff %" PRIu32 " ms", s_backoff_ms + jitter);
    sleep_ms_wdt(s_backoff_ms + jitter);
}

static TaskHandle_t s_task;

static void uplink_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    s_task = xTaskGetCurrentTaskHandle();

    int64_t next_hb_us    = 0;
    int64_t next_hello_us = 0;
    int64_t next_cmd_us   = 0;
    const int64_t hb_period_us    = (int64_t)cfg_heartbeat_interval_s() * 1000000;
    const int64_t hello_period_us = (int64_t)HELLO_INTERVAL_S * 1000000;
    const int64_t cmd_period_us   = (int64_t)CMD_POLL_INTERVAL_S * 1000000;
    const uint32_t idle_ms = cfg_upload_interval_s() * 1000;

    while (1) {
        esp_task_wdt_reset();
        /* acquisition keeps filling the spool while we wait here */
        EventBits_t bits = net_mgr_wait(NET_BIT_WIFI | NET_BIT_TIME,
                                        pdMS_TO_TICKS(5000));
        if (!(bits & NET_BIT_WIFI)) {
            continue;
        }

        /* /node/v1/hello: hoc/lam moi api_key + nguon gio du phong khi chua
         * co SNTP (server_time_ms) — chay ngay ca khi NET_BIT_TIME chua len,
         * do la duong DUY NHAT cung cap ca hai truoc khi SNTP dong bo. */
        if (esp_timer_get_time() >= next_hello_us) {
            send_hello();
            next_hello_us = esp_timer_get_time() + hello_period_us;
        }

        if (!(bits & NET_BIT_TIME)) {
            /* SNTP (pool.ntp.org) chay TRUOC — uu tien gio UTC chuan; hello
             * o tren da nap server_time_ms lam du phong neu SNTP cham/khong
             * co internet. Chua co gio hop le thi cho vong sau. */
            continue;
        }

        if (esp_timer_get_time() >= next_hb_us) {
            send_heartbeat();
            next_hb_us = esp_timer_get_time() + hb_period_us;
        }

        /* check_config_update() is NOT called here — see its own comment.
         * Under edge_collector/pcm_base, heartbeat's "config_version" is
         * pcm.edge.config_rev (the WHOLE edge's revision, bumped by any
         * channel on any device), not a per-node wiring version, and
         * /node/v1/config -> pcm.channel._as_config() carries no bus/host/
         * reg at all. Auto-applying it here was overwriting real local
         * channel wiring (e.g. an mbrtu sensor) with a host-less mbtcp
         * table and rebooting every time an unrelated device's edge_rev
         * ticked (incident: 2026-09-16, config v57->v59 in ~70s wiped a
         * hand-configured RS485 sensor). Re-enable only once pcm.channel
         * carries real physical wiring fields end to end. */

        if (esp_timer_get_time() >= next_cmd_us) {
            poll_command();
            next_cmd_us = esp_timer_get_time() + cmd_period_us;
        }

        /* server bao co lenh in cho (co trong ack/heartbeat response) */
        run_print_jobs();

        size_t n = spool_peek(s_batch, BATCH_MAX);
        if (n == 0) {
            /* idle sleep, but uplink_kick() (scale event) wakes us early */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(idle_ms));
            continue;
        }

        if (upload_batch(s_batch, n)) {
            s_backoff_ms = 0;
            /* full batch => likely draining a backlog: loop immediately */
            if (n < BATCH_MAX) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(idle_ms));
            }
        } else {
            backoff_sleep();
        }
    }
}

esp_err_t uplink_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(uplink_task, "uplink", 8192,
                                            NULL, 5, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

uint32_t uplink_sent_total(void)
{
    return __atomic_load_n(&s_sent_total, __ATOMIC_RELAXED);
}

bool uplink_server_ok(void)
{
    int64_t last = s_last_ok_us;
    if (last == 0) {
        return false;
    }
    int64_t window_us = (int64_t)cfg_upload_interval_s() * 3 * 1000000;
    return (esp_timer_get_time() - last) < window_us;
}

void uplink_set_tower_cb(uplink_tower_cb_t cb)
{
    s_tower_cb = cb;
}

void uplink_kick(void)
{
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

#endif /* CONFIG_UPLINK_HTTP_ENABLE */
