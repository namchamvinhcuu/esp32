/* mb_rtu: Modbus RTU master on the board's onboard RS485 transceiver
 * (Waveshare ESP32-S3-RS485-CAN: UART1 pins 17/18, isolated, automatic
 * direction switching — no DE/RE handling needed).
 *
 * Two consumers share the bus through one mutex:
 *   - the poller task reading "mbrtu" channels from channels.json
 *     (XY-MD02 class sensors), pushing measurements like mb_tcp does;
 *   - tower_light writing relay coils (Waveshare Modbus RTU Relay module)
 *     via mb_rtu_write_coils().
 *
 * Multi-drop reminder: every slave needs a unique address; XY-MD02 and the
 * Waveshare relay module BOTH ship as address 1 — change one of them. */
#ifndef FMS_MB_RTU_H
#define FMS_MB_RTU_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initializes the RS485 UART and starts the sensor poller when "mbrtu"
 * channels exist. Safe to call when nothing uses the bus (no-op). */
esp_err_t mb_rtu_start(void);

/* FC3 (holding) / FC4 (input): read n registers. FC1 (coil) / FC2
 * (discrete input): read n bits, out[i] = 0/1. ESP_ERR_INVALID_STATE
 * when the bus is not initialized, ESP_ERR_TIMEOUT / ESP_FAIL on comms. */
esp_err_t mb_rtu_read_regs(uint8_t slave, uint8_t func, uint16_t reg,
                           uint16_t n, uint16_t *out);

/* FC15: write n coils from bit array (bits[0] bit0 = first coil). */
esp_err_t mb_rtu_write_coils(uint8_t slave, uint16_t start, uint16_t n,
                             const uint8_t *bits);

#ifdef __cplusplus
}
#endif

#endif /* FMS_MB_RTU_H */
