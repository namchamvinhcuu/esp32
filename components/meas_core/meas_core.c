#include <inttypes.h>
#include <sys/time.h>

#include "esp_log.h"

#include "cfg.h"
#include "meas_core.h"

#define MEAS_QUEUE_LEN   256
#define PUSH_TIMEOUT_MS  20
#define MEAS_MAX_CHANNEL 16
#define MEAS_MAX_TAP     2

static const char *TAG = "meas";

static QueueHandle_t s_q;
static uint32_t s_seq;     /* incremented atomically, strictly increasing per boot */
static uint32_t s_dropped;

/* last value per channel for live consumers (BLE feed); torn reads are
 * prevented by the spinlock, cheap enough for the push hot path */
static measurement_t     s_latest[MEAS_MAX_CHANNEL];
static bool              s_latest_valid[MEAS_MAX_CHANNEL];
static portMUX_TYPE      s_latest_mux = portMUX_INITIALIZER_UNLOCKED;

/* dang ky mot lan luc khoi dong, doc tu hot path — khong khoa */
static meas_tap_t s_taps[MEAS_MAX_TAP];
static uint8_t    s_tap_n;

esp_err_t meas_add_tap(meas_tap_t cb)
{
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_tap_n >= MEAS_MAX_TAP) {
        return ESP_ERR_NO_MEM;
    }
    s_taps[s_tap_n++] = cb;
    return ESP_OK;
}

esp_err_t meas_core_init(void)
{
    s_q = xQueueCreate(MEAS_QUEUE_LEN, sizeof(measurement_t));
    if (s_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "queue ready (%d x %u bytes)", MEAS_QUEUE_LEN,
             (unsigned)sizeof(measurement_t));
    return ESP_OK;
}

static int64_t now_epoch_ms(void)
{
    /* SNTP sets system time; uplink gates on NET_BIT_TIME so pre-sync
     * timestamps never reach the server */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

bool meas_push(uint16_t channel_id, meas_source_t source,
               meas_quality_t quality, float value)
{
    measurement_t m = {
        .seq        = __atomic_add_fetch(&s_seq, 1, __ATOMIC_RELAXED),
        .boot_id    = cfg_boot_id(),
        .channel_id = channel_id,
        .source     = (uint8_t)source,
        .quality    = (uint8_t)quality,
        .ts_ms      = now_epoch_ms(),
        .value      = value,
    };
    if (channel_id < MEAS_MAX_CHANNEL) {
        /* live cache updates even when the queue is full — a BLE viewer
         * should still see fresh values while uploads are backed up */
        taskENTER_CRITICAL(&s_latest_mux);
        s_latest[channel_id]       = m;
        s_latest_valid[channel_id] = true;
        taskEXIT_CRITICAL(&s_latest_mux);
    }

    /* Goi tap TRUOC hang doi chinh: quan sat vien thay ca nhung mau bi
     * rot vi hang day, nen so lieu doi chieu khong bi lech ngam. */
    for (uint8_t i = 0; i < s_tap_n; i++) {
        s_taps[i](&m);
    }

    if (xQueueSend(s_q, &m, pdMS_TO_TICKS(PUSH_TIMEOUT_MS)) != pdTRUE) {
        __atomic_add_fetch(&s_dropped, 1, __ATOMIC_RELAXED);
        ESP_LOGW(TAG, "queue full, dropped ch=%u seq=%" PRIu32,
                 (unsigned)channel_id, m.seq);
        return false;
    }
    return true;
}

bool meas_latest(uint16_t channel_id, measurement_t *out)
{
    if (channel_id >= MEAS_MAX_CHANNEL) {
        return false;
    }
    bool valid;
    taskENTER_CRITICAL(&s_latest_mux);
    valid = s_latest_valid[channel_id];
    if (valid) {
        *out = s_latest[channel_id];
    }
    taskEXIT_CRITICAL(&s_latest_mux);
    return valid;
}

uint32_t meas_dropped_count(void)
{
    return __atomic_load_n(&s_dropped, __ATOMIC_RELAXED);
}

QueueHandle_t meas_queue(void)
{
    return s_q;
}
