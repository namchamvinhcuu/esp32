/* cfg: node identity (NVS) + channel table.
 * Channel table source order: server-provided JSON stored in NVS (see
 * cfg_store_channels_json) first, embedded channels.json as the factory
 * fallback. Seeds NVS from Kconfig CONFIG_FMS_* on first boot; later
 * changes go through NVS so a reflash is never needed to re-home the node. */
#ifndef FMS_CFG_H
#define FMS_CFG_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Tran so kenh toan cuc — MOI mang per-channel (bang cfg, next_due cua cac
 * poller, last_sent_seq cua BLE...) deu phai dung hang nay, khong hardcode. */
#define CFG_MAX_CHANNELS 32

typedef enum {
    BUS_MBTCP  = 0,
    BUS_SERIAL = 1,
    BUS_I2C    = 2,
    BUS_GPIO   = 3,
    BUS_MBRTU  = 4,   /* Modbus RTU on the onboard RS485 (unit = slave addr) */
    BUS_BLE    = 5,   /* BLE instrument (caliper...): host = MAC/name prefix */
} cfg_bus_t;

typedef enum {
    IO_COUNT  = 0,  /* pulse counter: value = total pulses since boot */
    IO_STATE  = 1,  /* level monitor: value = 0/1, pushed on change + period */
    IO_OUTPUT = 2,  /* relay/lamp output: driven by gpio_out_write(), never
                       scanned as an input (see io_scan.c) */
} cfg_io_mode_t;

typedef enum {
    EDGE_FALLING = 0,
    EDGE_RISING  = 1,
    EDGE_BOTH    = 2,
} cfg_edge_t;

typedef enum {
    DT_U16 = 0,
    DT_I16 = 1,
    DT_U32 = 2,
    DT_F32 = 3,
    DT_I32 = 4,
} cfg_dtype_t;

typedef struct {
    uint16_t    id;         /* index into the table == measurement_t.channel_id */
    char        code[16];   /* payload 'ch' key, e.g. "temp1" */
    char        name[32];   /* ten hien thi (BLE feed cho app); rong = code */
    cfg_bus_t   bus;
    char        unit_name[8];
    /* local alarm thresholds (any bus); NAN = not set. Breach -> RED tower */
    float       alarm_high;
    float       alarm_low;
    /* bus == BUS_MBTCP */
    char        host[32];
    uint16_t    port;
    uint8_t     unit;       /* Modbus unit id */
    uint8_t     func;       /* 1 = coil, 2 = discrete input (value 0/1),
                               3 = holding, 4 = input register */
    uint16_t    reg;
    cfg_dtype_t dtype;
    bool        word_swap;  /* 32-bit types: device sends loword first (CDAB) */
    float       scale;
    float       offset;
    uint32_t    period_ms;
    /* bus == BUS_SERIAL */
    uint8_t     uart;
    uint32_t    baud;
    char        parser[16];
    uint8_t     databits;       /* 7 or 8 (default 8) */
    char        parity;         /* 'N' | 'E' | 'O' (default 'N') */
    uint8_t     stopbits;       /* 1 or 2 (default 1) */
    char        poll_cmd[16];   /* raw bytes sent to the device; "" = the
                                   device streams by itself (measureCommand
                                   of the old Python edge config) */
    uint32_t    poll_period_ms; /* how often to send poll_cmd; 0 = never */
    /* bus == BUS_I2C (cheap demo sensors on the shared I2C bus) */
    char        chip[12];   /* "sht3x" | "aht20" */
    uint8_t     addr;       /* 7-bit address; 0 = chip default */
    char        meas[8];    /* "temp" | "humid" */
    /* bus == BUS_GPIO (counting / state inputs through opto isolators) */
    uint8_t       gpio_pin;
    cfg_io_mode_t io_mode;
    cfg_edge_t    edge;
    uint32_t      debounce_ms; /* lockout after an accepted edge */
    bool          invert;      /* state mode: report !level */
    bool          pull_down;   /* default internal pull-up */
} cfg_channel_t;

esp_err_t cfg_init(void);

/* Channel table parsed from NVS-stored config or the embedded channels.json
 * (never NULL after cfg_init; *count may be 0 if the JSON is unusable). */
const cfg_channel_t *cfg_get_channels(size_t *count);
const cfg_channel_t *cfg_channel_by_id(uint16_t id);

/* Lookup by 'code' (payload 'ch' / server command's "channel" string).
 * NULL if no channel has that code. Used by the /node/v1/commands executor
 * (uplink.c) to resolve a server-sent channel name back to wiring. */
const cfg_channel_t *cfg_channel_by_code(const char *code);

/* Validate and persist a server /config response body to NVS; the table is
 * applied on the NEXT boot (caller reboots). Rejects unparseable JSON, a
 * missing/invalid config_version or channels array, or entries without a
 * code — the running config is never clobbered by a bad download. */
esp_err_t cfg_store_channels_json(const char *json, size_t len);

/* Erase the NVS-stored server /config blob (falls back to the embedded
 * channels.json on the next call to cfg_get_channels source resolution,
 * i.e. next boot). Does not touch WiFi/server/api_key/boot_id — surgical
 * recovery from a bad server-pushed channel table without losing
 * provisioning. ESP_ERR_NVS_NOT_FOUND if nothing was stored. */
esp_err_t cfg_clear_stored_channels(void);

uint32_t    cfg_config_version(void);
uint32_t    cfg_upload_interval_s(void);
uint32_t    cfg_heartbeat_interval_s(void);

const char *cfg_get_server_url(void); /* no trailing slash */
const char *cfg_get_api_key(void);

/* Persist an api_key learned from the edge's /node/v1/hello response (Odoo
 * has since created a pcm.device for this serial) — see uplink's
 * send_hello(). No-op if unchanged; takes effect immediately, no reboot
 * needed. */
esp_err_t cfg_set_api_key(const char *api_key);
const char *cfg_wifi_ssid(void);
const char *cfg_wifi_pass(void);

/* 12 uppercase hex chars from the eFuse station MAC, e.g. "84F703A1B2C4" */
const char *cfg_node_serial(void);

/* NVS boot counter, incremented once per boot; payload 'bid' */
uint16_t    cfg_boot_id(void);

/* --- provisioning (SoftAP config portal) support --- */

/* Web login password for the portal (NVS, seeded from Kconfig). */
const char *cfg_web_pass(void);

/* Human-readable install location ("Máy ép số 3"), set from the portal;
 * sent in heartbeats so the server can name the device on first contact. */
const char *cfg_loc_name(void);

/* True when the node should boot into the SoftAP portal instead of normal
 * operation: WiFi never configured (still the Kconfig placeholder) or the
 * force flag was set (BOOT button); the force flag is cleared by this call. */
bool cfg_take_provisioning_request(void);

/* Set the force flag so the NEXT boot enters the portal (used by the
 * BOOT-button watcher; caller reboots afterwards). */
esp_err_t cfg_request_provisioning(void);

/* Persist bootstrap config from the portal. NULL/empty fields keep the
 * currently stored value (except pass, which follows ssid). */
esp_err_t cfg_store_bootstrap(const char *ssid, const char *pass,
                              const char *url, const char *apikey,
                              const char *webpass_new, const char *locname);

#ifdef __cplusplus
}
#endif

#endif /* FMS_CFG_H */
