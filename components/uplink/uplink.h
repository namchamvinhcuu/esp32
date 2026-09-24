/* uplink: talks to edge_collector's /node/v1/... contract (see
 * fms-node/node_agent/node_agent/edge_client.py + node_api.py for the
 * reference implementation this firmware mirrors): hello (learns api_key +
 * server time), measurements, heartbeat, config, commands. Gates on
 * NET_BIT_WIFI; exponential backoff with jitter on failure. */
#ifndef FMS_UPLINK_H
#define FMS_UPLINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t uplink_start(void);

/* Samples acknowledged by the server since boot. */
uint32_t uplink_sent_total(void);

/* True while the last successful server exchange is recent (~3 upload
 * intervals). Used by tower_light for the GREEN/health lamp. */
bool uplink_server_ok(void);

/* Wake the uploader immediately instead of waiting for the periodic tick.
 * Called by EVENT sources (scale weighings) so business records reach the
 * server in ~1 s rather than one upload interval later. Safe anywhere. */
void uplink_kick(void);

/* API responses may carry {"tower":{"r":..,"y":..,"g":..,"b":..,"bz":..}};
 * uplink forwards each occurrence to this callback (values 0/1, or -1 when
 * a key is absent). Registered by tower_light at startup for a
 * server-driven override on top of its own local alarm logic.
 * Currently inert: edge_collector's /node/v1/... responses never include a
 * "tower" key (no equivalent of the old server-push exists there yet) —
 * kept so wiring that back in later is a small change, not a rewrite. */
typedef void (*uplink_tower_cb_t)(int r, int y, int g, int b, int bz);
void uplink_set_tower_cb(uplink_tower_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* FMS_UPLINK_H */
