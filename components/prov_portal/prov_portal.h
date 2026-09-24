/* prov_portal: SoftAP captive-portal configuration page.
 *
 * Entry paths:
 *   - fresh node (WiFi still the Kconfig placeholder)  -> portal at boot
 *   - BOOT button held ~5 s during normal operation    -> flag + reboot
 *
 * In portal mode the node is maintenance-only (no acquisition/uplink):
 *   AP "FMS-NODE-xxxx" (WPA2, CONFIG_FMS_PROV_AP_PASS) at 192.168.4.1,
 *   DNS answering everything with 192.168.4.1 (captive popup),
 *   web page guarded by a login password (CONFIG_FMS_WEB_PASS / NVS).
 * Saving credentials live-tests the target WiFi (APSTA), persists to NVS
 * on success and reboots into normal mode. */
#ifndef FMS_PROV_PORTAL_H
#define FMS_PROV_PORTAL_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Run the portal. Never returns (the portal reboots the node on save). */
void prov_portal_run(void);

/* Normal-mode variant: serve the same login-protected config page on the
 * node's station IP (http://<node-ip>/) while everything else keeps
 * running. Saving persists to NVS and reboots; the live WiFi join test is
 * skipped (empty SSID keeps the current WiFi). No SoftAP, no captive DNS. */
esp_err_t prov_portal_start_runtime(void);

/* Normal-mode helper: watch the BOOT button; on a ~5 s hold set the
 * provisioning flag and reboot into the portal. */
esp_err_t prov_button_watch_start(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_PROV_PORTAL_H */
