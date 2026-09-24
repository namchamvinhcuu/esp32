#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "cfg.h"

/* components/cfg/channels.json embedded by this component's CMakeLists
 * (EMBED_TXTFILES, NUL-terminated). */
extern const char _binary_channels_json_start[];

#define NVS_NS       "fms"
#define NVS_CHJSON   "chjson"   /* server /config body, applied on boot */
#define CHJSON_MAX   16384      /* ~300 B/kenh x 32 kenh + du phong */
#define MAX_CHANNELS CFG_MAX_CHANNELS

static const char *TAG = "cfg";

static cfg_channel_t s_channels[MAX_CHANNELS];
static size_t        s_channel_count;
static uint32_t      s_config_version = 1;
static uint32_t      s_upload_interval_s = 10;
static uint32_t      s_heartbeat_interval_s = 60;

static char     s_server_url[128];
static char     s_api_key[64];
static char     s_wifi_ssid[33];
static char     s_wifi_pass[65];
static char     s_web_pass[33];
static char     s_loc_name[48];
static char     s_serial[13];
static uint16_t s_boot_id;
static bool     s_force_prov;

/* Read a string key; if absent, seed NVS with the Kconfig default. */
static void nvs_str_or_seed(nvs_handle_t h, const char *key,
                            const char *seed, char *out, size_t out_len)
{
    size_t len = out_len;
    esp_err_t err = nvs_get_str(h, key, out, &len);
    if (err != ESP_OK) {
        strlcpy(out, seed, out_len);
        ESP_ERROR_CHECK(nvs_set_str(h, key, out));
        ESP_LOGI(TAG, "seeded %s from Kconfig", key);
    }
}

static cfg_dtype_t parse_dtype(const char *s)
{
    if (s == NULL)             return DT_U16;
    if (strcmp(s, "i16") == 0) return DT_I16;
    if (strcmp(s, "u32") == 0) return DT_U32;
    if (strcmp(s, "i32") == 0) return DT_I32;
    if (strcmp(s, "f32") == 0) return DT_F32;
    return DT_U16;
}

static void json_str(const cJSON *obj, const char *key, char *out, size_t out_len)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(it) && it->valuestring != NULL) {
        strlcpy(out, it->valuestring, out_len);
    }
}

/* copy s into out translating "\r" "\n" "\t" escapes into real bytes, so
 * channels.json can express poll commands like "SI\r\n" */
static void unescape_into(const char *s, char *out, size_t out_len)
{
    size_t o = 0;
    for (const char *p = s; *p != '\0' && o < out_len - 1; p++) {
        if (*p == '\\' && p[1] != '\0') {
            p++;
            out[o++] = (*p == 'r') ? '\r' :
                       (*p == 'n') ? '\n' :
                       (*p == 't') ? '\t' : *p;
        } else {
            out[o++] = *p;
        }
    }
    out[o] = '\0';
}

static double json_num(const cJSON *obj, const char *key, double dflt)
{
    const cJSON *it = cJSON_GetObjectItemCaseSensitive(obj, key);
    return cJSON_IsNumber(it) ? it->valuedouble : dflt;
}

/* Parse a config JSON (embedded channels.json or a stored server /config
 * body) into the channel table. Returns false without touching globals on
 * a malformed document so the caller can fall back to the other source. */
static bool parse_channels(const char *json, const char *src)
{
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGE(TAG, "config (%s) parse error: %s", src, cJSON_GetErrorPtr());
        return false;
    }
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "channels");
    if (!cJSON_IsArray(arr)) {
        ESP_LOGE(TAG, "config (%s): no channels array", src);
        cJSON_Delete(root);
        return false;
    }

    s_channel_count        = 0;
    s_config_version       = (uint32_t)json_num(root, "config_version", 1);
    s_upload_interval_s    = (uint32_t)json_num(root, "upload_interval_s", 10);
    s_heartbeat_interval_s = (uint32_t)json_num(root, "heartbeat_interval_s", 60);

    const cJSON *ch;
    cJSON_ArrayForEach(ch, arr) {
        if (s_channel_count >= MAX_CHANNELS) {
            ESP_LOGW(TAG, "channel table full (%d), rest ignored", MAX_CHANNELS);
            break;
        }
        cfg_channel_t *c = &s_channels[s_channel_count];
        memset(c, 0, sizeof(*c));
        c->id = (uint16_t)s_channel_count;

        json_str(ch, "code", c->code, sizeof(c->code));
        json_str(ch, "name", c->name, sizeof(c->name));
        if (c->name[0] == '\0') {
            strlcpy(c->name, c->code, sizeof(c->name)); /* rong -> dung code */
        }
        /* name di vao JSON feed BLE bang snprintf (khong qua cJSON):
         * vo hieu ky tu pha JSON */
        for (char *p = c->name; *p != '\0'; p++) {
            if (*p == '"' || *p == '\\') {
                *p = '\'';
            }
        }
        json_str(ch, "unit_name", c->unit_name, sizeof(c->unit_name));
        c->alarm_high = (float)json_num(ch, "alarm_high", NAN);
        c->alarm_low  = (float)json_num(ch, "alarm_low", NAN);

        char bus[12] = "mbtcp";
        json_str(ch, "bus", bus, sizeof(bus));
        if (strcmp(bus, "serial") == 0) {
            c->bus  = BUS_SERIAL;
            c->uart = (uint8_t)json_num(ch, "uart", 1);
            c->baud = (uint32_t)json_num(ch, "baud", CONFIG_FMS_SCALE_BAUD);
            strlcpy(c->parser, "cas_generic", sizeof(c->parser));
            json_str(ch, "parser", c->parser, sizeof(c->parser));

            /* frame format, default 8N1 (older scales often run 7E1) */
            c->databits = (uint8_t)json_num(ch, "databits", 8);
            c->stopbits = (uint8_t)json_num(ch, "stopbits", 1);
            char par[8] = "none";
            json_str(ch, "parity", par, sizeof(par));
            c->parity = (par[0] == 'e' || par[0] == 'E') ? 'E' :
                        (par[0] == 'o' || par[0] == 'O') ? 'O' : 'N';

            /* poll command for request/response scales; "" = streaming */
            char raw_cmd[24] = "";
            json_str(ch, "poll_cmd", raw_cmd, sizeof(raw_cmd));
            unescape_into(raw_cmd, c->poll_cmd, sizeof(c->poll_cmd));
            c->poll_period_ms = (uint32_t)json_num(ch, "poll_period_ms",
                                                   c->poll_cmd[0] ? 1000 : 0);
        } else if (strcmp(bus, "mbrtu") == 0) {
            c->bus       = BUS_MBRTU;
            /* "slave" preferred; "unit" accepted (Odoo /config emits both) */
            c->unit      = (uint8_t)json_num(ch, "slave",
                                             json_num(ch, "unit", 1));
            c->func      = (uint8_t)json_num(ch, "func", 4);
            c->reg       = (uint16_t)json_num(ch, "reg", 0);
            c->scale     = (float)json_num(ch, "scale", 1.0);
            c->offset    = (float)json_num(ch, "offset", 0.0);
            c->period_ms = (uint32_t)json_num(ch, "period_ms", 2000);
            char dt[8] = "u16";
            json_str(ch, "dtype", dt, sizeof(dt));
            c->dtype = parse_dtype(dt);
            c->word_swap = cJSON_IsTrue(
                cJSON_GetObjectItemCaseSensitive(ch, "word_swap"));
        } else if (strcmp(bus, "gpio") == 0) {
            c->bus      = BUS_GPIO;
            c->gpio_pin = (uint8_t)json_num(ch, "gpio",
                                            json_num(ch, "pin", 15));
            char mode[8] = "count";
            json_str(ch, "mode", mode, sizeof(mode));
            c->io_mode = (strcmp(mode, "state") == 0)  ? IO_STATE :
                         (strcmp(mode, "output") == 0) ? IO_OUTPUT : IO_COUNT;
            char edge[8] = "falling";
            json_str(ch, "edge", edge, sizeof(edge));
            c->edge = (strcmp(edge, "rising") == 0) ? EDGE_RISING :
                      (strcmp(edge, "both") == 0)   ? EDGE_BOTH : EDGE_FALLING;
            c->debounce_ms = (uint32_t)json_num(ch, "debounce_ms", 10);
            /* OUTPUT: "invert" means active-low relay (IN pulled LOW to
             * close the relay) — same flag, read by gpio_out_write(). */
            c->invert    = cJSON_IsTrue(
                               cJSON_GetObjectItemCaseSensitive(ch, "invert"));
            char pull[8] = "up";
            json_str(ch, "pull", pull, sizeof(pull));
            c->pull_down = (strcmp(pull, "down") == 0);
            c->period_ms = (uint32_t)json_num(ch, "period_ms", 5000);
            c->scale     = (float)json_num(ch, "scale", 1.0);
            c->offset    = (float)json_num(ch, "offset", 0.0);
        } else if (strcmp(bus, "ble") == 0) {
            c->bus = BUS_BLE;
            /* host = dinh danh thiet bi: MAC "AA:BB:CC:DD:EE:FF" hoac
             * TIEN TO ten quang ba (vd "Caliper") — ble_central tu nhan
             * dang theo dinh dang chuoi */
            json_str(ch, "mac", c->host, sizeof(c->host));
            if (c->host[0] == '\0') {
                json_str(ch, "name", c->host, sizeof(c->host));
            }
            strlcpy(c->parser, "raw_line", sizeof(c->parser));
            json_str(ch, "parser", c->parser, sizeof(c->parser));
            c->scale  = (float)json_num(ch, "scale", 1.0);
            c->offset = (float)json_num(ch, "offset", 0.0);
            if (c->host[0] == '\0') {
                ESP_LOGW(TAG, "channel %s: ble khong co mac/name — "
                         "ble_central se chi chay che do QUET", c->code);
            }
        } else if (strcmp(bus, "i2c") == 0) {
            c->bus = BUS_I2C;
            strlcpy(c->chip, "sht3x", sizeof(c->chip));
            json_str(ch, "chip", c->chip, sizeof(c->chip));
            c->addr      = (uint8_t)json_num(ch, "addr", 0); /* 0 = chip default */
            strlcpy(c->meas, "temp", sizeof(c->meas));
            json_str(ch, "meas", c->meas, sizeof(c->meas));
            c->scale     = (float)json_num(ch, "scale", 1.0);
            c->offset    = (float)json_num(ch, "offset", 0.0);
            c->period_ms = (uint32_t)json_num(ch, "period_ms", 2000);
        } else {
            c->bus  = BUS_MBTCP;
            json_str(ch, "host", c->host, sizeof(c->host));
            c->port      = (uint16_t)json_num(ch, "port", 502);
            if (c->port == 0) {
                /* port 0 (vo le) -> mac dinh 502; tranh tao mot ket noi
                 * "phantom" (host:0) rieng lam nhieu pool va treo gateway */
                c->port = 502;
            }
            c->unit      = (uint8_t)json_num(ch, "unit", 1);
            c->func      = (uint8_t)json_num(ch, "func", 4);
            c->reg       = (uint16_t)json_num(ch, "reg", 0);
            c->scale     = (float)json_num(ch, "scale", 1.0);
            c->offset    = (float)json_num(ch, "offset", 0.0);
            c->period_ms = (uint32_t)json_num(ch, "period_ms", 2000);

            char dt[8] = "u16";
            json_str(ch, "dtype", dt, sizeof(dt));
            c->dtype = parse_dtype(dt);
            c->word_swap = cJSON_IsTrue(
                cJSON_GetObjectItemCaseSensitive(ch, "word_swap"));
            if (c->host[0] == '\0') {
                /* keep the channel (server may fill the host later) but the
                 * poller skips it — avoids a getaddrinfo("") retry loop */
                ESP_LOGW(TAG, "channel %s: mbtcp without host, poller will "
                         "skip it", c->code);
            }
        }

        if (c->code[0] == '\0') {
            ESP_LOGW(TAG, "channel %u has no code, skipped", (unsigned)c->id);
            continue;
        }
        s_channel_count++;
    }
    cJSON_Delete(root);
    ESP_LOGI(TAG, "config v%" PRIu32 " (%s): %u channels", s_config_version,
             src, (unsigned)s_channel_count);
    return true;
}

/* Server config stored by cfg_store_channels_json; malloc'd, NUL-terminated,
 * NULL when absent/oversized. */
static char *read_stored_channels(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return NULL;
    }
    size_t len = 0;
    esp_err_t err = nvs_get_blob(h, NVS_CHJSON, NULL, &len);
    if (err != ESP_OK || len == 0 || len > CHJSON_MAX) {
        nvs_close(h);
        return NULL;
    }
    char *buf = malloc(len + 1);
    if (buf == NULL) {
        nvs_close(h);
        return NULL;
    }
    err = nvs_get_blob(h, NVS_CHJSON, buf, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        free(buf);
        return NULL;
    }
    buf[len] = '\0';
    return buf;
}

static void load_channels(void)
{
    char *stored = read_stored_channels();
    if (stored != NULL) {
        bool ok = parse_channels(stored, "NVS/server");
        free(stored);
        if (ok) {
            return;
        }
        ESP_LOGW(TAG, "stored config unusable, falling back to embedded");
    }
    if (!parse_channels(_binary_channels_json_start, "embedded")) {
        s_channel_count = 0; /* node still boots: heartbeat/portal reachable */
    }
}

esp_err_t cfg_store_channels_json(const char *json, size_t len)
{
    if (json == NULL || len == 0 || len > CHJSON_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* Full validation pass before anything touches NVS. */
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        ESP_LOGE(TAG, "store: JSON parse error");
        return ESP_ERR_INVALID_ARG;
    }
    const cJSON *ver = cJSON_GetObjectItemCaseSensitive(root, "config_version");
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "channels");
    bool valid = cJSON_IsNumber(ver) && cJSON_IsArray(arr);
    int n_ch = 0;
    const cJSON *ch;
    cJSON_ArrayForEach(ch, arr) {
        const cJSON *code = cJSON_GetObjectItemCaseSensitive(ch, "code");
        if (!cJSON_IsObject(ch) || !cJSON_IsString(code) ||
            code->valuestring[0] == '\0') {
            valid = false;
            break;
        }
        n_ch++;
    }
    cJSON_Delete(root);
    if (!valid) {
        ESP_LOGE(TAG, "store: rejected (need config_version + channels[].code)");
        return ESP_ERR_INVALID_ARG;
    }
    if (n_ch > MAX_CHANNELS) {
        ESP_LOGW(TAG, "store: %d channels, only first %d will be used",
                 n_ch, MAX_CHANNELS);
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_CHJSON, json, len);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "server config stored (%u bytes, %d channels); "
                 "applies on next boot", (unsigned)len, n_ch);
    }
    return err;
}

esp_err_t cfg_clear_stored_channels(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h, NVS_CHJSON);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t cfg_init(void)
{
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    nvs_str_or_seed(h, "ssid",   CONFIG_FMS_WIFI_SSID,  s_wifi_ssid,  sizeof(s_wifi_ssid));
    nvs_str_or_seed(h, "pass",   CONFIG_FMS_WIFI_PASS,  s_wifi_pass,  sizeof(s_wifi_pass));
    nvs_str_or_seed(h, "url",    CONFIG_FMS_SERVER_URL, s_server_url, sizeof(s_server_url));
    nvs_str_or_seed(h, "apikey", CONFIG_FMS_API_KEY,    s_api_key,    sizeof(s_api_key));
    nvs_str_or_seed(h, "webpass", CONFIG_FMS_WEB_PASS,  s_web_pass,   sizeof(s_web_pass));
    nvs_str_or_seed(h, "locname", "",                    s_loc_name,   sizeof(s_loc_name));

    uint8_t force = 0;
    (void)nvs_get_u8(h, "force_prov", &force);
    if (force) {
        s_force_prov = true;
        ESP_ERROR_CHECK(nvs_set_u8(h, "force_prov", 0)); /* one-shot */
    }

    /* strip trailing slash so uplink can append /api/iot/v1/... verbatim */
    size_t ul = strlen(s_server_url);
    if (ul > 0 && s_server_url[ul - 1] == '/') {
        s_server_url[ul - 1] = '\0';
    }

    uint16_t boot = 0;
    (void)nvs_get_u16(h, "boot_id", &boot);
    boot++;
    ESP_ERROR_CHECK(nvs_set_u16(h, "boot_id", boot));

    /* ONE-TIME recovery, incident 2026-09-16: before uplink.c's auto-apply
     * of /node/v1/config was disabled, a pcm_base edge pushed a host-less
     * mbtcp channel table down and it got written here, clobbering real
     * local wiring (an mbrtu sensor). Clear the stale blob once so cfg
     * falls back to the embedded channels.json; safe to delete this block
     * (and the "chjson_rstN" key) in a later release once confirmed.
     *
     * 2026-09-21: bumped rst1 -> rst2. The blob survived that first reset
     * (it was re-stored afterwards) and it still pinned count1 to gpio1,
     * so moving count1 onto the pedal pin in channels.json changed nothing
     * — the boot log said "config v1 (NVS/server)" and "count1: gpio1".
     * A flash alone does NOT clear NVS; the key has to move to force it. */
    uint8_t chjson_rst = 0;
    (void)nvs_get_u8(h, "chjson_rst2", &chjson_rst);
    if (!chjson_rst) {
        esp_err_t rst_err = cfg_clear_stored_channels();
        ESP_LOGI(TAG, "one-time NVS channel-config reset: %s",
                 esp_err_to_name(rst_err));
        ESP_ERROR_CHECK(nvs_set_u8(h, "chjson_rst2", 1));
    }

    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    s_boot_id = boot;

    load_channels();

    ESP_LOGI(TAG, "serial=%s boot_id=%u server=%s", s_serial,
             (unsigned)s_boot_id, s_server_url);
    return ESP_OK;
}

const cfg_channel_t *cfg_get_channels(size_t *count)
{
    if (count != NULL) {
        *count = s_channel_count;
    }
    return s_channels;
}

const cfg_channel_t *cfg_channel_by_id(uint16_t id)
{
    return (id < s_channel_count) ? &s_channels[id] : NULL;
}

const cfg_channel_t *cfg_channel_by_code(const char *code)
{
    if (code == NULL || code[0] == '\0') {
        return NULL;
    }
    for (size_t i = 0; i < s_channel_count; i++) {
        if (strcmp(s_channels[i].code, code) == 0) {
            return &s_channels[i];
        }
    }
    return NULL;
}

uint32_t cfg_config_version(void)      { return s_config_version; }
uint32_t cfg_upload_interval_s(void)   { return s_upload_interval_s; }
uint32_t cfg_heartbeat_interval_s(void){ return s_heartbeat_interval_s; }
const char *cfg_get_server_url(void)   { return s_server_url; }
const char *cfg_get_api_key(void)      { return s_api_key; }

esp_err_t cfg_set_api_key(const char *api_key)
{
    if (api_key == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strlen(api_key) >= sizeof(s_api_key)) {
        /* would silently truncate the in-RAM cache below while NVS keeps
         * the full string — next boot's nvs_get_str(...,sizeof(s_api_key))
         * would then fail (ESP_ERR_NVS_INVALID_LENGTH) and reseed from the
         * Kconfig default, permanently losing the learned key. Reject
         * instead of drifting. */
        ESP_LOGE(TAG, "api_key from edge too long (%u >= %u), rejected",
                 (unsigned)strlen(api_key), (unsigned)sizeof(s_api_key));
        return ESP_ERR_INVALID_SIZE;
    }
    if (strcmp(api_key, s_api_key) == 0) {
        return ESP_OK; /* unchanged */
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, "apikey", api_key);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        strlcpy(s_api_key, api_key, sizeof(s_api_key));
    }
    return err;
}
const char *cfg_wifi_ssid(void)        { return s_wifi_ssid; }
const char *cfg_wifi_pass(void)        { return s_wifi_pass; }
const char *cfg_web_pass(void)         { return s_web_pass; }
const char *cfg_loc_name(void)         { return s_loc_name; }
const char *cfg_node_serial(void)      { return s_serial; }
uint16_t cfg_boot_id(void)             { return s_boot_id; }

bool cfg_take_provisioning_request(void)
{
    /* Unconfigured = SSID still the LITERAL Kconfig placeholder (must match
     * the default in Kconfig.projbuild). Comparing against the current
     * CONFIG_FMS_WIFI_SSID value would trap menuconfig-provisioned nodes in
     * the portal forever, since NVS is seeded from it on first boot. */
    return s_force_prov ||
           s_wifi_ssid[0] == '\0' ||
           strcmp(s_wifi_ssid, "changeme-ssid") == 0;
}

esp_err_t cfg_request_provisioning(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, "force_prov", 1);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t cfg_store_bootstrap(const char *ssid, const char *pass,
                              const char *url, const char *apikey,
                              const char *webpass_new, const char *locname)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (ssid != NULL && ssid[0] != '\0') {
        err |= nvs_set_str(h, "ssid", ssid);
        err |= nvs_set_str(h, "pass", pass != NULL ? pass : "");
    }
    if (url != NULL && url[0] != '\0') {
        err |= nvs_set_str(h, "url", url);
    }
    if (apikey != NULL && apikey[0] != '\0') {
        err |= nvs_set_str(h, "apikey", apikey);
    }
    if (webpass_new != NULL && webpass_new[0] != '\0') {
        err |= nvs_set_str(h, "webpass", webpass_new);
    }
    if (locname != NULL && locname[0] != '\0') {
        err |= nvs_set_str(h, "locname", locname);
    }
    err |= nvs_set_u8(h, "force_prov", 0);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err == ESP_OK ? ESP_OK : ESP_FAIL;
}
