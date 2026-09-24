/* mqtt_link: phat so do len broker MQTT, SONG SONG voi uplink HTTP.
 *
 * GIAI DOAN 2 cua viec doi duong truyen. Component nay KHONG cat gi cua
 * uplink.c: HTTP van la duong chinh thuc, van la ben duy nhat goi
 * spool_ack_through(). Muc dich cua giai doan nay la do — chay hai duong
 * canh nhau mot thoi gian, doi chieu so mau, roi moi cat HTTP.
 *
 * Vi sao khong doc tu spooler: spool_peek() chi tra ve nhung ban ghi CU
 * NHAT va uplink moi la ben xoa chung di (spool_ack_through). Neu MQTT
 * cung doc o day thi hai ben tranh nhau mot vong dem — khi HTTP ack truoc,
 * MQTT mat ban ghi, va so lieu doi chieu thanh vo nghia.
 *
 * Thay vao do: mot cai "tap" gan vao meas_core, thay moi so do NGAY luc no
 * sinh ra, truoc khi vao spooler. Hai duong doc tu cung mot nguon nhung
 * khong dung vao trang thai cua nhau.
 */
#ifndef FMS_MQTT_LINK_H
#define FMS_MQTT_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Khong lam gi va tra ESP_OK khi CONFIG_MQTT_LINK_ENABLE tat hoac khi
 * broker URI de trong — de mot node chua cau hinh van boot binh thuong. */
esp_err_t mqtt_link_start(void);

/* So so do da duoc broker xac nhan (PUBACK, QoS 1) ke tu luc boot.
 * Dem nay de so truc tiep voi uplink_sent_total() — hai con so phai
 * bam sat nhau, do chinh la phep kiem cua giai doan 2. */
uint32_t mqtt_link_published(void);

/* So so do bi bo vi hang doi day hoac vi dang mat ket noi broker.
 * O giai doan 2 mat mat o day KHONG lam mat du lieu that: HTTP van gui
 * du. Con so nay noi cho ta biet MQTT co theo kip khong. */
uint32_t mqtt_link_dropped(void);

/* So ban ghi bi bao-khi-doi nen lai (gia tri khong doi, chua toi han nhac
 * lai). Con so nay cho biet RBE dang tiet kiem bao nhieu — do 18/09 thi
 * count1/pedal1/count2 chiem 75% luu luong chi de noi "van la 0". */
uint32_t mqtt_link_suppressed(void);

bool mqtt_link_connected(void);

/* Thuc day tac vu phat ngay thay vi doi het chu ky cho. scale_serial
 * goi (qua uplink_kick) khi co lan can moi — ban ghi nghiep vu phai di
 * trong ~1 giay, khong doi nhip nhan roi. An toan o moi noi. */
void mqtt_link_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_MQTT_LINK_H */
