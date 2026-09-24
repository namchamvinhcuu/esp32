/* diag: dong ho do cua node — KHONG doi hanh vi cua bat cu thu gi.
 *
 * Vi sao can: dong "status:" cu chi co esp_get_free_heap_size() va
 * esp_get_minimum_free_heap_size(). Hai so do khong du, va con lam lac
 * huong:
 *
 *  - min_heap la vach thap nhat KE TU LUC BOOT va KHONG BAO GIO hoi lai.
 *    Do 20/09: heap dung yen o 35 980 suot 17 gio trong khi min_heap ket o
 *    25 748. Nhin vao min_heap thi tuong dang ro ri; that ra do la MOT lan
 *    tut nhat thoi da xay ra tu lau. Cai ta can biet la "cua so 10 giay vua
 *    roi tut sau nhat bao nhieu", va do la win_min o day.
 *
 *  - tong byte trong khong noi duoc co cap phat noi 4 KB hay khong. Bo nho
 *    vun thanh nhieu lo nho thi tong van dep ma malloc van truot. Nen phai
 *    doc them khoi trong LON NHAT.
 *
 * Bai hoc phai tra gia: truoc khi co nhung so nay toi da dua ra bon gia
 * thuyet sai ve bo nho cua node va lam dut chuyen hai lan. Do truoc, doan sau.
 */
#ifndef FMS_DIAG_H
#define FMS_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Goi moi ~100 ms. Chi lay mau heap de biet day cua so — re, khong khoa. */
void diag_tick(void);

/* Goi moi 10 giay, ngay sau dong "status:". In mot dong heap; cu lan thu
 * `every` (xem TASKS_EVERY trong diag.c) thi in them bang tac vu. */
void diag_report(void);

#ifdef __cplusplus
}
#endif

#endif /* FMS_DIAG_H */
