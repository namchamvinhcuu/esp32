/* io_scan: digital inputs through opto isolators (24V factory signals).
 *
 * Two channel modes (channels.json, bus "gpio"):
 *
 *   COUNT - pulse counting (proximity/photo sensor, press stroke contact):
 *     {"code":"press_count","bus":"gpio","mode":"count","gpio":15,
 *      "edge":"falling","debounce_ms":10,"period_ms":5000}
 *     value reported every period = TOTAL pulses since boot (monotonic;
 *     the server/dashboard derives rates and per-shift deltas).
 *
 *   STATE - level monitoring (machine run contact, existing tower lamp):
 *     {"code":"may1_run","bus":"gpio","mode":"state","gpio":16,
 *      "invert":false,"debounce_ms":50,"period_ms":10000}
 *     value 0/1 pushed on every (debounced) change and every period.
 *
 * Rates: ISR + lockout debounce is solid to a few hundred Hz — plenty for
 * production counting. TODO(encoder): switch to the PCNT peripheral for
 * kHz-range encoders. */
#ifndef FMS_IO_SCAN_H
#define FMS_IO_SCAN_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t io_scan_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_IO_SCAN_H */
