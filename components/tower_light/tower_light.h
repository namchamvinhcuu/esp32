/* tower_light: drives a plain DC signal tower (Qisen LTA-504 class: one
 * wire per color + buzzer) through a relay/transistor module.
 *
 * Local logic, evaluated every 500 ms:
 *   GREEN  steady - node healthy: WiFi up, time synced, server reachable
 *   YELLOW        - degraded: any channel in COMM_ERR, or WiFi/server down
 *   RED           - alarm: any channel value beyond its alarm_high/alarm_low
 *   BLUE          - unused locally (server's to command)
 *   BUZZER        - off locally by default (Kconfig option to tie to RED)
 *
 * Server override: API responses may carry {"tower":{"r":0,"y":0,"g":1,
 * "b":0,"bz":0}}; uplink forwards it here and it REPLACES local logic for
 * TOWER_OVERRIDE_TTL_MS, then local logic resumes. */
#ifndef FMS_TOWER_LIGHT_H
#define FMS_TOWER_LIGHT_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TOWER_OVERRIDE_TTL_MS 90000

esp_err_t tower_light_start(void);

/* -1 for a color = keep the local value for that color; 0/1 force it. */
void tower_override(int r, int y, int g, int b, int bz);

/* For the config-page status block. */
bool tower_output_ok(void);           /* MBRTU: last coil write acked */
const char *tower_backend_name(void); /* "mbrtu" | "gpio" | "off" */

#ifdef __cplusplus
}
#endif

#endif /* FMS_TOWER_LIGHT_H */
