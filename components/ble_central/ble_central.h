/* ble_central: doc thiet bi do BLE (thuoc kep, can BLE...) — vai CENTRAL
 * chay SONG SONG voi ble_uart (peripheral, feed app B4A) tren cung mot
 * NimBLE host. KHONG init NimBLE o day — ble_uart_start() phai chay truoc.
 *
 * v1 (khung, 07/2026 — chua co thiet bi that):
 *  - Muc tieu = kenh BUS_BLE dau tien trong config; host = MAC
 *    "AA:BB:CC:DD:EE:FF" hoac TIEN TO ten quang ba.
 *  - Quet + log MOI thiet bi xung quanh (che do scanner phuc vu do UUID
 *    khi hang ve); khop muc tieu -> ket noi -> tim characteristic NOTIFY
 *    (uu tien FFE1/NUS-TX, khong co thi lay cai dau tien) -> subscribe.
 *  - Parser "raw_line": gom byte notify thanh dong, bat cum so dau tien
 *    -> meas_push + uplink_kick. Khung binary la se hexdump de viet parser
 *    rieng sau (mo hinh scale_parse).
 *  - Rot ket noi -> tu quet lai (backoff 5s). */
#ifndef FMS_BLE_CENTRAL_H
#define FMS_BLE_CENTRAL_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Goi SAU ble_uart_start(). Khong co kenh BUS_BLE thi la no-op. */
esp_err_t ble_central_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_BLE_CENTRAL_H */
