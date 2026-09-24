/* FMS Node entry point.
 *
 * Demo scope: WiFi STA + Modbus TCP polling + serial scale + HTTP uplink
 * with RAM spool (survives WiFi loss, NOT reboot).
 *
 * Intentionally NOT in this demo (upgrade path):
 *  - OTA:       add otadata/ota_0/ota_1 partitions + esp_https_ota
 *  - LittleFS:  swap spooler ring internals for a flash segment spool
 *  - BLE/CAN:   not needed for demo hardware
 *  - RS485 RTU: swap mb_tcp for espressif/esp-modbus
 */
#include <inttypes.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

#include "ble_central.h"
#include "ble_print.h"
#include "ble_uart.h"
#include "cfg.h"
#include "diag.h"
#include "gpio_out.h"
#include "i2c_sens.h"
#include "io_scan.h"
#include "meas_core.h"
#include "prov_portal.h"
#include "net_mgr.h"
#include "mb_rtu.h"
#include "mb_tcp.h"
#include "mqtt_link.h"
#include "scale_serial.h"
#include "spooler.h"
#include "tower_light.h"
#include "uplink.h"

static const char *TAG = "main";

static int64_t epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void app_main(void)
{
    /* NVS is required by cfg (boot counter, creds) and by WiFi */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(cfg_init());

    /* Fresh node or BOOT button held -> maintenance-only SoftAP portal
     * (FMS-NODE-xxxx). Saves credentials to NVS then reboots; no
     * acquisition runs in this mode. */
    if (cfg_take_provisioning_request()) {
        prov_portal_run(); /* never returns */
    }

    ESP_ERROR_CHECK(meas_core_init());
    ESP_ERROR_CHECK(net_mgr_start());
    ESP_ERROR_CHECK(spooler_init());
    ESP_ERROR_CHECK(mb_tcp_start());
    ESP_ERROR_CHECK(mb_rtu_start());  /* onboard RS485: sensors + tower relay */
    ESP_ERROR_CHECK(i2c_sens_start());
    ESP_ERROR_CHECK(io_scan_start());
    ESP_ERROR_CHECK(gpio_out_start()); /* relay/lamp outputs, server-commanded */
    ESP_ERROR_CHECK(scale_serial_start());
    ESP_ERROR_CHECK(uplink_start());
    ESP_ERROR_CHECK(mqtt_link_start());
    ESP_ERROR_CHECK(ble_uart_start());   /* live JSON feed for the B4A app */
    ESP_ERROR_CHECK(ble_central_start()); /* BLE instruments (caliper...) */
    ESP_ERROR_CHECK(ble_print_init());   /* pool may in BLE (noi thuong truc) */
    ESP_ERROR_CHECK(tower_light_start()); /* Qisen tower via relay module */
    ESP_ERROR_CHECK(prov_button_watch_start()); /* BOOT 5 s -> portal */
    ESP_ERROR_CHECK(prov_portal_start_runtime()); /* http://<node-ip>/ */

    ESP_LOGI(TAG, "fms-node up: serial=%s boot_id=%u fw=0.1.0",
             cfg_node_serial(), (unsigned)cfg_boot_id());

    /* Vong chinh nhip 100 ms: bao cao van 10 giay mot lan nhu cu, nhung
     * giua hai lan bao cao no lay mau heap de biet cua so vua roi tut sau
     * bao nhieu. KHONG tao tac vu rieng cho viec do — mot tac vu moi la
     * mot ngan xep moi lay ngay trong vung nho ma ta dang muon do. */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    unsigned tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
        diag_tick();
        if (++tick < 100) {
            continue;
        }
        tick = 0;
        /* `sent` la uplink_sent_total(); khi CONFIG_UPLINK_HTTP_ENABLE tat
         * thi ham do tra ve chinh mqtt_pub, nen dong nay van doc duoc o ca
         * hai cau hinh. `mq` la trang thai broker — voi HTTP tat, do la
         * dieu kien duy nhat de den XANH sang. */
        ESP_LOGI(TAG,
                 "status: heap=%" PRIu32 " min_heap=%" PRIu32
                 " rssi=%d spool=%u sent=%" PRIu32 " drop=%" PRIu32
                 " mq=%d pub=%" PRIu32 " mqdrop=%" PRIu32 " rbe=%" PRIu32
                 " epoch=%" PRId64,
                 esp_get_free_heap_size(),
                 esp_get_minimum_free_heap_size(),
                 net_mgr_rssi(),
                 (unsigned)spool_depth(),
                 uplink_sent_total(),
                 meas_dropped_count(),
                 mqtt_link_connected() ? 1 : 0,
                 mqtt_link_published(),
                 mqtt_link_dropped(),
                 mqtt_link_suppressed(),
                 /* 21/09: in dong ho cua node. Odoo bao so do "cu 2-3 giay"
                  * trong khi do duong truyen chi 0,3 s — chenh lech do lam
                  * bang "gia tri qua cu" chop tat va day ca bo cuc. Muon biet
                  * lech bao nhieu thi phai doc duoc dong ho o day. */
                 epoch_ms());
        diag_report();
    }
}
