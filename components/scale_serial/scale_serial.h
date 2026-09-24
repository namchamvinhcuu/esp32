/* scale_serial: UART reader + line assembler for the serial scale.
 * !!! The scale's RS-232 port is +/-12V — a MAX3232 level shifter between
 * the DB9 and the ESP32 pins is MANDATORY (see docs/demo-plan.md). */
#ifndef FMS_SCALE_SERIAL_H
#define FMS_SCALE_SERIAL_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the UART reader task if a BUS_SERIAL channel is configured.
 * Report policy: stable value pushed on change >= 0.01 or every 5 s;
 * unstable readings pushed at most once per second with Q_UNSTABLE. */
esp_err_t scale_serial_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_SCALE_SERIAL_H */
