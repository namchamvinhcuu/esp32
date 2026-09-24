#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mdns.h"

#include "cfg.h"
#include "net_mgr.h"

#define BACKOFF_MIN_MS 1000
#define BACKOFF_MAX_MS 60000

static const char *TAG = "net";

static EventGroupHandle_t s_eg;
static esp_timer_handle_t s_reconnect_timer;
static uint32_t           s_backoff_ms = BACKOFF_MIN_MS;
static int64_t            s_time_offset_ms; /* 0 until first SNTP sync */
static bool               s_sntp_started;
static char               s_ip[16];

static void reconnect_cb(void *arg)
{
    ESP_LOGI(TAG, "reconnecting...");
    esp_wifi_connect();
}

static void schedule_reconnect(void)
{
    /* exponential backoff with 0-30% jitter so a fleet power-cycling
     * together does not hammer the AP in lockstep */
    uint32_t jitter = s_backoff_ms / 100 * (esp_random() % 31);
    uint64_t delay_us = (uint64_t)(s_backoff_ms + jitter) * 1000;
    esp_timer_start_once(s_reconnect_timer, delay_us);
    ESP_LOGW(TAG, "wifi down, retry in %" PRIu32 " ms", s_backoff_ms + jitter);
    s_backoff_ms *= 2;
    if (s_backoff_ms > BACKOFF_MAX_MS) {
        s_backoff_ms = BACKOFF_MAX_MS;
    }
}

static void sntp_synced_cb(struct timeval *tv)
{
    if (s_time_offset_ms == 0) {
        struct timeval now;
        gettimeofday(&now, NULL);
        int64_t epoch_ms  = (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000;
        int64_t uptime_ms = esp_timer_get_time() / 1000;
        s_time_offset_ms = epoch_ms - uptime_ms;
        ESP_LOGI(TAG, "time synced, offset=%lld ms", (long long)s_time_offset_ms);
    }
    xEventGroupSetBits(s_eg, NET_BIT_TIME);
}

static void start_sntp_once(void)
{
    if (s_sntp_started) {
        return;
    }
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    cfg.sync_cb = sntp_synced_cb;
    ESP_ERROR_CHECK(esp_netif_sntp_init(&cfg));
    s_sntp_started = true;
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = data;
        xEventGroupClearBits(s_eg, NET_BIT_WIFI);
        ESP_LOGW(TAG, "disconnected, reason=%d", d ? d->reason : -1);
        schedule_reconnect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "got ip %s rssi=%d — config page: http://%s/",
                 s_ip, net_mgr_rssi(), s_ip);
        s_backoff_ms = BACKOFF_MIN_MS;
        xEventGroupSetBits(s_eg, NET_BIT_WIFI);
        start_sntp_once();
    }
}

esp_err_t net_mgr_start(void)
{
    s_eg = xEventGroupCreate();
    if (s_eg == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t targs = {
        .callback = reconnect_cb,
        .name     = "wifi_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_reconnect_timer));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();

    /* DHCP hostname: node shows up as "FMS-NODE-xxxx" in the customer's
     * router client list and in IP scanners, instead of a nameless
     * "Espressif Inc." entry */
    static char hostname[16];
    const char *serial = cfg_node_serial();
    size_t sl = strlen(serial);
    snprintf(hostname, sizeof(hostname), "FMS-NODE-%s",
             sl >= 4 ? serial + sl - 4 : serial);
    esp_netif_set_hostname(sta, hostname);

    /* mDNS: stable name regardless of the DHCP-assigned IP.
     * Browse to http://fms-node-xxxx.local/ from any device on the LAN. */
    char mdns_name[16];
    for (size_t i = 0; i <= strlen(hostname); i++) {
        mdns_name[i] = (hostname[i] >= 'A' && hostname[i] <= 'Z')
                           ? hostname[i] + 32 : hostname[i];
    }
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(mdns_name));
    mdns_instance_name_set("FMS datalogger node");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mdns: http://%s.local/", mdns_name);

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, cfg_wifi_ssid(), sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, cfg_wifi_pass(), sizeof(wc.sta.password));
    /* stationary node: pick the strongest AP for the SSID, never pin BSSID */
    wc.sta.scan_method        = WIFI_ALL_CHANNEL_SCAN;
    wc.sta.sort_method        = WIFI_CONNECT_AP_BY_SIGNAL;
    wc.sta.threshold.rssi     = -75;
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* mains powered: modem sleep only adds latency and coex trouble */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "sta start, ssid=%s", cfg_wifi_ssid());
    return ESP_OK;
}

EventBits_t net_mgr_wait(EventBits_t bits, TickType_t timeout)
{
    return xEventGroupWaitBits(s_eg, bits, pdFALSE, pdTRUE, timeout);
}

EventBits_t net_mgr_bits(void)
{
    return xEventGroupGetBits(s_eg);
}

int net_mgr_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

int64_t net_mgr_time_offset_ms(void)
{
    return s_time_offset_ms;
}

void net_mgr_time_from_server(int64_t epoch_ms)
{
    if (epoch_ms <= 0 || (xEventGroupGetBits(s_eg) & NET_BIT_TIME)) {
        return; /* already synced (SNTP or an earlier server response) */
    }
    struct timeval tv = {
        .tv_sec  = (time_t)(epoch_ms / 1000),
        .tv_usec = (suseconds_t)((epoch_ms % 1000) * 1000),
    };
    settimeofday(&tv, NULL);
    if (s_time_offset_ms == 0) {
        s_time_offset_ms = epoch_ms - esp_timer_get_time() / 1000;
    }
    ESP_LOGW(TAG, "time set from SERVER (%lld ms) — SNTP unavailable, "
             "accuracy follows the server clock", (long long)epoch_ms);
    xEventGroupSetBits(s_eg, NET_BIT_TIME);
}

const char *net_mgr_ip(void)
{
    return s_ip;
}
