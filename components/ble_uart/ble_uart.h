/* ble_uart: BLE peripheral exposing the Nordic UART Service (NUS) so a
 * phone/tablet (e.g. the B4A app in <repo>/b4a/) can subscribe and receive
 * live measurements as JSON lines:
 *   {"ch":"temp1","v":24.60,"q":0,"ts":1752130800000}\n
 *
 * Advertised name: "FMS-" + last 4 hex of the node serial (MAC).
 * Notifies are chunked to the negotiated ATT MTU; the client reassembles
 * on '\n'. One client at a time (CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1).
 * Bytes written by the client to the RX characteristic are logged only. */
#ifndef FMS_BLE_UART_H
#define FMS_BLE_UART_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_uart_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_BLE_UART_H */
