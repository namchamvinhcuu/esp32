/* i2c_sens: cheap demo sensors on one shared I2C bus (SDA/SCL via Kconfig).
 * Supported chips:
 *   sht3x  (SHT30/31/35 modules, default addr 0x44) - temp + humidity
 *   aht20  (AHT20/AHT21 modules,  default addr 0x38) - temp + humidity
 * One channels.json entry per VALUE, e.g. a single SHT30 board provides:
 *   {"code":"temp_demo","bus":"i2c","chip":"sht3x","meas":"temp","period_ms":2000}
 *   {"code":"humid_demo","bus":"i2c","chip":"sht3x","meas":"humid","period_ms":2000}
 * Wiring (defaults): VCC->3V3, GND->GND, SDA->GPIO8, SCL->GPIO9. */
#ifndef FMS_I2C_SENS_H
#define FMS_I2C_SENS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the polling task if any BUS_I2C channel is configured. */
esp_err_t i2c_sens_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_I2C_SENS_H */
