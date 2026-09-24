/* spooler: durable-ish measurement buffer between acquisition and uplink.
 *
 * Demo backend: RAM ring (survives WiFi loss, NOT reboot). The API is
 * deliberately shaped so a LittleFS segment-file spool (see design doc)
 * can replace the internals without touching uplink:
 *   append -> peek(batch) -> upload -> ack_through(acked seq).
 */
#ifndef FMS_SPOOLER_H
#define FMS_SPOOLER_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "meas_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Allocates the ring (PSRAM when available, else internal RAM) and starts
 * the drain task that moves records from the meas_core queue into the ring. */
esp_err_t spooler_init(void);

size_t spool_depth(void);

/* Copy up to max_n oldest records into out, WITHOUT removing them.
 * Returns the number copied. Oldest-first == ascending seq. */
size_t spool_peek(measurement_t *out, size_t max_n);

/* Drop all buffered records of boot_id with seq <= seq (server acked them). */
void spool_ack_through(uint16_t boot_id, uint32_t seq);

/* Records lost to ring overflow (oldest overwritten) since boot. */
uint32_t spool_overwritten_count(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_SPOOLER_H */
