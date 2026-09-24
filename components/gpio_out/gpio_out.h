/* gpio_out: CHU SO HUU DUY NHAT cua cac chan GPIO output (channels.json /
 * server config, bus="gpio" mode="output"). Moi kenh la mot duong relay /
 * den / van: {"code":"relay_blue","bus":"gpio","gpio":6,"mode":"output",
 * "invert":false}.
 *
 * "Chu so huu duy nhat" la phan quan trong, va no den tu mot loi that.
 *
 * Truoc 19/09/2026, tower_light goi thang gpio_set_level() len dung nhung
 * chan nay, va cu 10 giay lai EP GHI LAI toan bo theo trang thai canh bao
 * cua rieng no. Nen bat mot den qua Odoo thi trong vong 10 giay no tu tat —
 * hien tuong "den sang mot luc roi tat" ma khong ai giai thich duoc. Te hon:
 * tower_light ghi thang ra chan nen KHONG day mot ban ghi nao len Odoo, vay
 * la bang so lieu van bao "dang sang" trong khi chan da ve 0. Mot cai bang
 * noi doi mot cach hoan toan nhat quan la thu kho tim nhat.
 *
 * Cung do la ly do den xanh duong khong sang: relay_blue la GPIO 6, ma
 * tower_light coi GPIO 6 la VANG. Bon tren nam chan trung nhau va ten bi
 * xao het.
 *
 * Nen bay gio moi thu di qua day. tower_light xin ghi bang
 * gpio_out_auto_write_pin(), va bi TU CHOI khi kenh do dang duoc nguoi giu.
 */
#ifndef FMS_GPIO_OUT_H
#define FMS_GPIO_OUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cau hinh moi kenh bus=="gpio" mode=="output" thanh GPIO output va keo ve
 * TAT (co tinh dao cuc), roi chay tac vu nhip 50 ms lo viec chop va het
 * gio. An toan khi khong co kenh nao (khong lam gi, tra ESP_OK).
 *
 * PHAI goi truoc tower_light_start(). */
esp_err_t gpio_out_start(void);

/* Mot lenh thiet bi tu may chu. Gom cac truong o day thay vi tam doi so
 * roi: tung nay da la gioi han de con doc duoc, va them mot kieu mau moi
 * sau nay khong phai sua chu ky ham o ba noi. */
typedef struct {
    const char *channel;    /* ma kenh, vd "relay_blue" */
    const char *op;         /* write | on | off | blink | identify */
    bool        has_value;  /* op=="write": co truong value khong */
    double      value;      /* op=="write": khac 0 la bat */
    int32_t     ms;         /* thoi luong; 0 = giu mai */
    int32_t     period_ms;  /* op=="blink": chu ky mot lan sang-tat */
} gpio_cmd_t;

/* Thuc thi mot lenh. `detail` nhan ly do khi tra false.
 *
 * De o DAY chu khong o uplink/mqtt_link vi tu 19/09 co HAI duong mang lenh
 * toi. Hai ban sao cua cung mot logic kiem tra la cach chac chan nhat de
 * chung lech nhau. */
bool gpio_out_execute(const gpio_cmd_t *cmd, char *detail, size_t detail_sz);

/* Duong cho logic TU DONG tai cho (tower_light) ghi ra mot chan.
 *
 * Nhuong nguoi: neu chan nay thuoc mot kenh dang trong han "nguoi dieu
 * khien" thi lan ghi nay bi BO QUA. Het han thi tu dong lay lai quyen —
 * de khong ai quen tra ma den bao dong nam im vinh vien.
 *
 * Tra true nghia la "chan nay cua toi, da xu ly" (du co thuc su ghi hay
 * khong). Tra false thi chan do khong thuoc kenh nao — vd chan xanh duong
 * cua thap — va ben goi tu ghi lay, bang dung quy uoc cuc cua rieng no.
 * Chia nhu vay de kien thuc ve cuc tinh nam o dung mot cho moi ben: o day
 * la ch->invert, ben kia la CONFIG_FMS_TOWER_ACTIVE_LOW. */
bool gpio_out_auto_write_pin(int pin, int on);

#ifdef __cplusplus
}
#endif

#endif /* FMS_GPIO_OUT_H */
