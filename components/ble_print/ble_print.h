/* ble_print: in nhan qua may in nhiet BLE (Richta RI-8002B va ho hang).
 *
 * KET NOI THUONG TRUC, NHIEU MAY IN (pool MAX_PRINTERS con):
 *   - Lan dau gap mot may in (tu job): quet -> ket noi -> do characteristic
 *     ghi (uu tien 0x2AF1, du phong 49535343-8841-... kieu ISSC) ~5-8s,
 *     roi GIU ket noi + nho dia chi vao NVS.
 *   - Cac job sau: bom thang ~0.3s (khong quet, khong do lai).
 *   - May in rot / node vua reboot: JOB KE TIEP tu noi lai ngay trong
 *     ble_print_run (noi thang dia chi da hoc, khong quet) — cham hon
 *     ~2-3s cho dung tem do, fail thi bam Retry. Khong co task nen.
 * Ngan sach ket noi: NIMBLE_MAX_CONNECTIONS=6 (dien thoai + thuoc kep +
 * toi da 4 may in). */
#ifndef FMS_BLE_PRINT_H
#define FMS_BLE_PRINT_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Goi mot lan sau khi NimBLE da init (ble_uart_start): tao pool, doc danh
 * sach may in da quen tu NVS, chay task tu noi lai. */
esp_err_t ble_print_init(void);

/* In `len` byte ra may in `mac` ("DD:0D:30:1F:E1:99", khong phan biet hoa
 * thuong). Dong bo — nhanh (~0.3s) khi da ket noi san, cham (~5-8s) lan
 * lam quen dau tien. Tra ESP_OK khi toan bo bytes da ghi xong. */
esp_err_t ble_print_run(const char *mac, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FMS_BLE_PRINT_H */
