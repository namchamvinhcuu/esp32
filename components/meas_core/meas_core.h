/* meas_core: central measurement type, sequence allocation, and the
 * producer queue every acquisition component pushes into. */
#ifndef FMS_MEAS_CORE_H
#define FMS_MEAS_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SRC_MBTCP  = 0,
    SRC_SERIAL = 1,
    SRC_I2C    = 2,
    SRC_GPIO   = 3,
    SRC_BLE    = 4,
} meas_source_t;

/* quality codes match the API contract 'q' field */
typedef enum {
    Q_GOOD         = 0,
    Q_COMM_ERR     = 1,
    Q_OUT_OF_RANGE = 2,
    Q_UNSTABLE     = 3,
} meas_quality_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;        /* strictly increasing per boot, allocated here */
    uint16_t boot_id;    /* from cfg; (serial,bid,seq) is the server dedup key */
    uint16_t channel_id; /* index into cfg channel table */
    uint8_t  source;     /* meas_source_t */
    uint8_t  quality;    /* meas_quality_t */
    int64_t  ts_ms;      /* epoch milliseconds UTC (valid after SNTP sync) */
    float    value;
} measurement_t; /* 22 bytes packed */

esp_err_t meas_core_init(void);

/* Timestamp + seq are filled in here. 20 ms send timeout; on timeout the
 * sample is dropped and the drop counter incremented. Safe from any task. */
bool meas_push(uint16_t channel_id, meas_source_t source,
               meas_quality_t quality, float value);

uint32_t meas_dropped_count(void);

/* Copy the most recent measurement of a channel (regardless of upload state).
 * Returns false if the channel has produced nothing yet. Used by ble_uart
 * for the live BLE feed. */
bool meas_latest(uint16_t channel_id, measurement_t *out);

/* Tap: quan sat vien cua MOI so do, goi ngay trong meas_push sau khi seq
 * va ts da duoc gan. Chay tren task do nen ham callback PHAI khong chan —
 * day mot hang doi rieng roi tra ve ngay.
 *
 * Co mat de mqtt_link phat song song voi uplink ma khong phai tranh
 * spooler: hai duong doc cung mot nguon, khong dung vao trang thai cua
 * nhau. Toi da MEAS_MAX_TAP quan sat vien. */
typedef void (*meas_tap_t)(const measurement_t *m);
esp_err_t meas_add_tap(meas_tap_t cb);

/* Consumed by the spooler drain task only. */
QueueHandle_t meas_queue(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_MEAS_CORE_H */
