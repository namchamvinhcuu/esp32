/* wdt_util: sleep dai ma van nuoi task watchdog (30 s timeout, xem
 * sdkconfig.defaults). Dung cho backoff/reconnect-wait co the keo dai hon
 * 1 giay — KHONG dung cho vong lap ngan (<=1s moi vong) da tu an toan chi
 * bang mot esp_task_wdt_reset() moi vong, xem uplink.c/mqtt_link.c cho vi
 * du dung dung cho, va cac task scan/poll ngan (mb_tcp/io_scan/i2c_sens/
 * mb_rtu/scale_serial/ble_print) khong can ham nay. */
#ifndef FMS_WDT_UTIL_H
#define FMS_WDT_UTIL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Ngu du ms mili-giay, chia thanh cac buoc <=1000 ms va goi
 * esp_task_wdt_reset() giua moi buoc — task GOI HAM NAY phai tu
 * esp_task_wdt_add() truoc (ham nay khong tu dang ky). */
void wdt_safe_sleep_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* FMS_WDT_UTIL_H */
