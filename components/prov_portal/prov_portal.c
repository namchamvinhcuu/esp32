#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "cfg.h"
#include "meas_core.h"
#include "net_mgr.h"
#include "prov_portal.h"
#include "spooler.h"
#include "tower_light.h"
#include "uplink.h"

static const char *TAG = "portal";

extern const char _binary_portal_html_start[];

#define PORTAL_IP        "192.168.4.1"
#define LOGIN_FAIL_DELAY 1500
#define STA_JOIN_RETRIES 2

/* target-WiFi live test state, polled by the page via /api/status */
typedef enum {
    T_IDLE, T_CONNECTING, T_GOT_IP, T_FAILED, T_REBOOTING
} test_state_t;

static volatile test_state_t s_test  = T_IDLE;
static volatile int  s_fail_reason;
static char  s_sta_ip[16];
static int   s_join_attempts;

/* pending values from /api/save, persisted only after the join succeeds */
static char s_p_ssid[33], s_p_pass[65], s_p_url[128];
static char s_p_apikey[64], s_p_webpass[33], s_p_locname[48];

static char s_ap_name[16];
static char s_token[33];          /* session cookie, random per portal run */
static esp_timer_handle_t s_reboot_timer;
static bool s_runtime_mode;       /* true = serving on station IP while running */

/* ------------------------------------------------------------- helpers -- */

static void reboot_cb(void *arg) { esp_restart(); }

static void schedule_reboot(uint32_t ms)
{
    s_test = T_REBOOTING;
    esp_timer_start_once(s_reboot_timer, (uint64_t)ms * 1000);
}

static void persist_pending_and_reboot(void)
{
    if (cfg_store_bootstrap(s_p_ssid, s_p_pass, s_p_url, s_p_apikey,
                            s_p_webpass, s_p_locname) == ESP_OK) {
        ESP_LOGI(TAG, "bootstrap saved, rebooting into normal mode");
        schedule_reboot(1500);
    } else {
        ESP_LOGE(TAG, "NVS write failed");
        s_test = T_FAILED;
        s_fail_reason = -1;
    }
}

/* --------------------------------------------------------- WiFi events -- */

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = data;
        if (s_test == T_CONNECTING) {
            if (++s_join_attempts <= STA_JOIN_RETRIES) {
                esp_wifi_connect();
            } else {
                s_fail_reason = d ? d->reason : -1;
                s_test = T_FAILED;
                ESP_LOGW(TAG, "join failed, reason=%d", s_fail_reason);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "target wifi OK, ip=%s", s_sta_ip);
        s_test = T_GOT_IP;
        persist_pending_and_reboot();
    }
}

static void start_sta_join(void)
{
    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, s_p_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, s_p_pass, sizeof(wc.sta.password));
    wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wc.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    s_join_attempts = 0;
    s_sta_ip[0] = '\0';
    s_test = T_CONNECTING;
    esp_wifi_connect();
}

/* -------------------------------------------------- captive DNS server -- */

/* answer every A query with 192.168.4.1 so the OS pops the portal page */
static void dns_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (sock < 0 || bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "dns bind failed");
        vTaskDelete(NULL);
        return;
    }

    uint8_t buf[512];
    while (1) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int len = recvfrom(sock, buf, sizeof(buf) - 16, 0,
                           (struct sockaddr *)&from, &flen);
        if (len < 12) {
            continue;
        }
        buf[2] = 0x81; buf[3] = 0x80;      /* response, recursion available */
        buf[6] = 0x00; buf[7] = 0x01;      /* answer count = 1 */
        buf[8] = buf[9] = buf[10] = buf[11] = 0;

        /* answer: pointer to name at 0x0C, A IN TTL=60 len=4 + our IP */
        uint8_t ans[] = { 0xC0, 0x0C, 0x00, 0x01, 0x00, 0x01,
                          0x00, 0x00, 0x00, 0x3C, 0x00, 0x04,
                          192, 168, 4, 1 };
        memcpy(buf + len, ans, sizeof(ans));
        sendto(sock, buf, len + sizeof(ans), 0,
               (struct sockaddr *)&from, flen);
    }
}

/* -------------------------------------------------------- HTTP helpers -- */

static bool req_authed(httpd_req_t *req)
{
    char cookie[96];
    if (httpd_req_get_hdr_value_str(req, "Cookie", cookie,
                                    sizeof(cookie)) != ESP_OK) {
        return false;
    }
    char *p = strstr(cookie, "FMSAUTH=");
    if (p == NULL) {
        return false;
    }
    p += 8;
    /* constant-time-ish compare over the full token length */
    uint8_t diff = 0;
    for (size_t i = 0; i < sizeof(s_token) - 1; i++) {
        diff |= (uint8_t)p[i] ^ (uint8_t)s_token[i];
        if (p[i] == '\0') {
            return false;
        }
    }
    return diff == 0;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    cJSON_free(body);
    return err;
}

static cJSON *read_json_body(httpd_req_t *req)
{
    char buf[512];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(buf)) {
        return NULL;
    }
    int got = 0;
    while (got < total) {
        int n = httpd_req_recv(req, buf + got, total - got);
        if (n <= 0) {
            return NULL;
        }
        got += n;
    }
    buf[total] = '\0';
    return cJSON_Parse(buf);
}

static void json_field(cJSON *o, const char *key, char *out, size_t out_len)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(o, key);
    if (cJSON_IsString(it) && it->valuestring != NULL) {
        strlcpy(out, it->valuestring, out_len);
    }
}

/* ------------------------------------------------------- HTTP handlers -- */

static esp_err_t h_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, _binary_portal_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

/* OS connectivity probes + any unknown URL -> redirect to the portal page.
 * Captive popups need the absolute AP address; in runtime mode a relative
 * redirect keeps whatever host/IP the browser used. */
static esp_err_t h_redirect(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location",
                       s_runtime_mode ? "/" : "http://" PORTAL_IP "/");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t h_login(httpd_req_t *req)
{
    cJSON *body = read_json_body(req);
    char pass[64] = "";
    if (body != NULL) {
        json_field(body, "password", pass, sizeof(pass));
        cJSON_Delete(body);
    }
    if (strlen(pass) == 0 || strcmp(pass, cfg_web_pass()) != 0) {
        vTaskDelay(pdMS_TO_TICKS(LOGIN_FAIL_DELAY)); /* brute-force brake */
        httpd_resp_set_status(req, "403 Forbidden");
        cJSON *r = cJSON_CreateObject();
        cJSON_AddBoolToObject(r, "ok", false);
        return send_json(req, r);
    }
    char cookie[64];
    snprintf(cookie, sizeof(cookie), "FMSAUTH=%s; Path=/", s_token);
    httpd_resp_set_hdr(req, "Set-Cookie", cookie);
    cJSON *r = cJSON_CreateObject();
    cJSON_AddBoolToObject(r, "ok", true);
    return send_json(req, r);
}

static esp_err_t h_status(httpd_req_t *req)
{
    cJSON *r = cJSON_CreateObject();
    bool authed = req_authed(req);
    cJSON_AddBoolToObject(r, "authed", authed);
    cJSON_AddBoolToObject(r, "runtime", s_runtime_mode);
    if (authed) {
        static const char *names[] = {
            "idle", "connecting", "got_ip", "failed", "rebooting"
        };
        cJSON_AddStringToObject(r, "node", s_ap_name);
        cJSON_AddStringToObject(r, "serial", cfg_node_serial());
        cJSON_AddStringToObject(r, "fw", "0.1.0");
        cJSON_AddStringToObject(r, "cur_ssid", cfg_wifi_ssid());
        cJSON_AddStringToObject(r, "cur_url", cfg_get_server_url());
        cJSON_AddStringToObject(r, "cur_name", cfg_loc_name());

        /* network identity: runtime = the station IP (net_mgr); setup
         * mode = the join-test result if any */
        cJSON_AddStringToObject(r, "ip",
                                s_runtime_mode ? net_mgr_ip() : s_sta_ip);
        char mdns_name[24];
        size_t o = 0;
        for (const char *p = s_ap_name; *p && o < sizeof(mdns_name) - 8; p++) {
            mdns_name[o++] = (*p >= 'A' && *p <= 'Z') ? *p + 32 : *p;
        }
        strcpy(&mdns_name[o], ".local");
        cJSON_AddStringToObject(r, "mdns", mdns_name);

        if (s_runtime_mode) {
            /* live health block (these subsystems only run in normal mode) */
            cJSON *h = cJSON_AddObjectToObject(r, "health");
            cJSON_AddBoolToObject(h, "srv_ok", uplink_server_ok());
            cJSON_AddNumberToObject(h, "sent", uplink_sent_total());
            cJSON_AddNumberToObject(h, "spool", (double)spool_depth());
            cJSON_AddNumberToObject(h, "drop", meas_dropped_count());
            cJSON_AddNumberToObject(h, "rssi", net_mgr_rssi());
            cJSON_AddStringToObject(h, "tower", tower_backend_name());
            cJSON_AddBoolToObject(h, "tower_ok", tower_output_ok());
        }
        cJSON_AddStringToObject(r, "test", names[s_test]);
        cJSON_AddNumberToObject(r, "fail_reason", s_fail_reason);
        cJSON_AddStringToObject(r, "sta_ip", s_sta_ip);
    }
    return send_json(req, r);
}

static esp_err_t h_scan(httpd_req_t *req)
{
    if (!req_authed(req)) {
        httpd_resp_set_status(req, "403 Forbidden");
        return httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }

    wifi_scan_config_t sc = { .show_hidden = false };
    esp_err_t err = esp_wifi_scan_start(&sc, true); /* blocks ~1.5 s */
    cJSON *r = cJSON_CreateObject();
    cJSON *arr = cJSON_AddArrayToObject(r, "networks");
    if (err == ESP_OK) {
        uint16_t n = 0;
        esp_wifi_scan_get_ap_num(&n);
        if (n > 20) {
            n = 20;
        }
        static wifi_ap_record_t recs[20];
        esp_wifi_scan_get_ap_records(&n, recs);
        for (uint16_t i = 0; i < n; i++) {
            cJSON *o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "ssid", (const char *)recs[i].ssid);
            cJSON_AddNumberToObject(o, "rssi", recs[i].rssi);
            cJSON_AddBoolToObject(o, "secure",
                                  recs[i].authmode != WIFI_AUTH_OPEN);
            cJSON_AddItemToArray(arr, o);
        }
    }
    return send_json(req, r);
}

/* body: {ssid, pass, url, apikey?, webpass?, skip_test?} */
static esp_err_t h_save(httpd_req_t *req)
{
    if (!req_authed(req)) {
        httpd_resp_set_status(req, "403 Forbidden");
        return httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }
    cJSON *body = read_json_body(req);
    if (body == NULL) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{}", HTTPD_RESP_USE_STRLEN);
    }
    s_p_ssid[0] = s_p_pass[0] = s_p_url[0] = '\0';
    s_p_apikey[0] = s_p_webpass[0] = s_p_locname[0] = '\0';
    json_field(body, "ssid", s_p_ssid, sizeof(s_p_ssid));
    json_field(body, "pass", s_p_pass, sizeof(s_p_pass));
    json_field(body, "url", s_p_url, sizeof(s_p_url));
    json_field(body, "apikey", s_p_apikey, sizeof(s_p_apikey));
    json_field(body, "webpass", s_p_webpass, sizeof(s_p_webpass));
    json_field(body, "locname", s_p_locname, sizeof(s_p_locname));
    bool skip = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(body, "skip_test"));
    cJSON_Delete(body);

    if (s_p_ssid[0] == '\0' && !s_runtime_mode) {
        /* setup mode has no stored credentials to fall back on */
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"ssid required\"}",
                               HTTPD_RESP_USE_STRLEN);
    }

    if (skip || s_runtime_mode) {
        /* runtime mode: node is already on WiFi; empty ssid keeps the
         * current credentials, everything persists and we reboot */
        persist_pending_and_reboot();
    } else {
        start_sta_join();
    }
    cJSON *r = cJSON_CreateObject();
    cJSON_AddBoolToObject(r, "ok", true);
    return send_json(req, r);
}

/* ----------------------------------------------------------- bring-up -- */

static void start_wifi_apsta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_evt, NULL));

    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, s_ap_name, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(s_ap_name);
    strlcpy((char *)ap.ap.password, CONFIG_FMS_PROV_AP_PASS,
            sizeof(ap.ap.password));
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.max_connection = 4;
    ap.ap.channel = 1;
    if (strlen(CONFIG_FMS_PROV_AP_PASS) < 8) {
        ap.ap.authmode = WIFI_AUTH_OPEN; /* WPA2 needs >= 8 chars */
        ESP_LOGW(TAG, "AP password too short, falling back to OPEN");
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static const httpd_uri_t URIS[] = {
    { .uri = "/",                    .method = HTTP_GET,  .handler = h_root },
    { .uri = "/api/login",           .method = HTTP_POST, .handler = h_login },
    { .uri = "/api/status",          .method = HTTP_GET,  .handler = h_status },
    { .uri = "/api/scan",            .method = HTTP_GET,  .handler = h_scan },
    { .uri = "/api/save",            .method = HTTP_POST, .handler = h_save },
    /* connectivity probes of Android/iOS/Windows */
    { .uri = "/generate_204",        .method = HTTP_GET,  .handler = h_redirect },
    { .uri = "/gen_204",             .method = HTTP_GET,  .handler = h_redirect },
    { .uri = "/hotspot-detect.html", .method = HTTP_GET,  .handler = h_redirect },
    { .uri = "/connecttest.txt",     .method = HTTP_GET,  .handler = h_redirect },
    { .uri = "/ncsi.txt",            .method = HTTP_GET,  .handler = h_redirect },
    { .uri = "/*",                   .method = HTTP_GET,  .handler = h_redirect },
};

static void init_identity(void)
{
    const char *serial = cfg_node_serial();
    size_t sl = strlen(serial);
    snprintf(s_ap_name, sizeof(s_ap_name), "FMS-NODE-%s",
             sl >= 4 ? serial + sl - 4 : serial);
    snprintf(s_token, sizeof(s_token), "%08" PRIx32 "%08" PRIx32
             "%08" PRIx32 "%08" PRIx32,
             esp_random(), esp_random(), esp_random(), esp_random());

    const esp_timer_create_args_t targs = {
        .callback = reboot_cb, .name = "prov_reboot",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_reboot_timer));
}

static esp_err_t start_http_server(void)
{
    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    /* Do 20/09: moi ket noi HTTP ton ~7 KB, ma may chu mac dinh cho 7 cai
     * cung luc — 3 yeu cau song song da keo heap trong tu 29 580 xuong
     * 8 560. Mot trinh duyet mo toi 6 ket noi song song toi cung mot may
     * chu, nghia la MO TRANG NAY BANG CHROME du de node het bo nho. Chan
     * o day chu khong o cho khac: day la con so duy nhat gioi han tong.
     *
     * lru_purge_enable dang bat, nen ket noi thu tu bi dong bot chu khong
     * lam hong yeu cau. Trang cau hinh cham hon mot chut doi lay chuyen
     * khong dung — doi thang.
     *
     * stack_size: do that sau khi da chay qua CA duong sau — POST
     * /api/login (read_json_body + cJSON), /api/status, va /api/scan co
     * xac thuc (quet wifi that) — dinh 2 140 byte. 4096 la du, nhung de
     * 6144: h_save la handler DUY NHAT khong do duoc (no ghi NVS roi khoi
     * dong lai node, khong dam chay tren chuyen dang song), va trang nay
     * chinh la duong cuu ho khi node sai cau hinh. 2 KB tren tong 42 KB
     * la gia re de khong phai panic o dung cai cong cu dung de sua loi. */
    hc.max_open_sockets = 3;
    hc.stack_size = 6144;
    hc.max_uri_handlers = 12;
    hc.uri_match_fn = httpd_uri_match_wildcard;
    hc.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &hc);
    if (err != ESP_OK) {
        return err;
    }
    for (size_t i = 0; i < sizeof(URIS) / sizeof(URIS[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &URIS[i]));
    }
    return ESP_OK;
}

void prov_portal_run(void)
{
    init_identity();
    start_wifi_apsta();
    xTaskCreate(dns_task, "prov_dns", 3072, NULL, 5, NULL);
    ESP_ERROR_CHECK(start_http_server());

    ESP_LOGI(TAG, "== PROVISIONING MODE ==");
    ESP_LOGI(TAG, "AP: %s  pass: %s  page: http://" PORTAL_IP "/",
             s_ap_name, CONFIG_FMS_PROV_AP_PASS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); /* portal runs until save+reboot */
    }
}

esp_err_t prov_portal_start_runtime(void)
{
    s_runtime_mode = true;
    init_identity();
    esp_err_t err = start_http_server();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "config page live at http://<node-ip>/ (login required)");
    }
    return err;
}

/* ------------------------------------------------- BOOT button watcher -- */

static void button_task(void *arg)
{
    const gpio_num_t pin = CONFIG_FMS_PROV_BUTTON_GPIO;
    int held_ms = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (gpio_get_level(pin) == 0) { /* BOOT is active-low */
            held_ms += 100;
            if (held_ms >= 5000) {
                ESP_LOGW(TAG, "BOOT held 5 s -> rebooting into portal");
                cfg_request_provisioning();
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            held_ms = 0;
        }
    }
}

esp_err_t prov_button_watch_start(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_FMS_PROV_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    BaseType_t ok = xTaskCreate(button_task, "prov_btn", 2560, NULL, 3, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
