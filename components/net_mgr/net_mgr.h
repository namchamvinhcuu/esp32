/* net_mgr: WiFi STA with backoff reconnect + SNTP. Downstream tasks gate on
 * the event group bits; nothing upstream ever checks connectivity. */
#ifndef FMS_NET_MGR_H
#define FMS_NET_MGR_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NET_BIT_WIFI BIT0 /* got IP */
#define NET_BIT_TIME BIT1 /* first SNTP sync done */

esp_err_t net_mgr_start(void);

/* Block until all requested bits are set (or timeout). Returns current bits. */
EventBits_t net_mgr_wait(EventBits_t bits, TickType_t timeout);

/* Non-blocking snapshot of the current bits. */
EventBits_t net_mgr_bits(void);

/* Current STA RSSI in dBm, 0 when not associated. */
int net_mgr_rssi(void);

/* Current station IP as text ("192.168.5.63"), "" before DHCP. Sent in
 * heartbeats so the server dashboard can link to the node's config page. */
const char *net_mgr_ip(void);

/* epoch_ms - uptime_ms captured at first SNTP sync; 0 before sync.
 * Used by uplink to repair timestamps recorded before the first sync
 * (pre-sync gettimeofday starts near the 1970 epoch + uptime). */
int64_t net_mgr_time_offset_ms(void);

/* Fallback clock source for networks WITHOUT internet (no pool.ntp.org):
 * uplink feeds the server_time_ms of a heartbeat response here. No-op once
 * NET_BIT_TIME is set (first SNTP sync stays authoritative). */
void net_mgr_time_from_server(int64_t epoch_ms);

#ifdef __cplusplus
}
#endif

#endif /* FMS_NET_MGR_H */
