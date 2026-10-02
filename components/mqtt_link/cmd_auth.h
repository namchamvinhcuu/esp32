#pragma once

/* Phần THUẦN C của đường lệnh ký (không FreeRTOS/ESP-IDF) — tách riêng để
 * test được bằng gcc trên máy dev (tests/host/test_cmd_auth.c).
 *
 * Format thống nhất 2026-10-02 với edge_collector (manager.py::queue_command,
 * mqtt_consumer.py::publish_command) và node_agent (mqtt_uplink.py::verify):
 *   - gói lệnh ký được edge publish ĐÚNG bằng canonical bytes của Python
 *     json.dumps(sort_keys=True, separators=(",",":")), nên phía C KHÔNG dựng
 *     lại canonical: cắt đúng một chuỗi con `,"sig":"<64 hex>"` khỏi raw
 *     bytes, phần còn lại chính là thông điệp HMAC-SHA256(api_key).
 *   - ack do node tự viết thẳng ở dạng canonical (key đã sort) để edge
 *     re-canonical ra đúng chuỗi đó rồi xác minh sig.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CMD_SIG_HEX_LEN 64

/* Tách sig khỏi gói lệnh thô.
 *
 * Tìm `,"sig":"` — trong JSON canonical dấu `"` nằm trong chuỗi luôn bị
 * escape thành `\"`, nên `,"` chỉ xuất hiện ở tầng cấu trúc, không thể bị
 * giả từ bên trong một giá trị chuỗi. Bắt buộc ĐÚNG MỘT lần xuất hiện,
 * theo sau là đúng 64 ký tự hex thường rồi `"`.
 *
 * Thành công: out = raw bỏ chuỗi con đó (NUL-terminated, *out_len = độ dài),
 * sig_hex = 64 hex + NUL. Trả false nếu không có sig, có >1 sig, sig sai
 * hình dạng, hoặc out không đủ chỗ. */
bool cmd_sig_strip(const char *raw, size_t raw_len,
                   char *out, size_t out_cap, size_t *out_len,
                   char sig_hex[CMD_SIG_HEX_LEN + 1]);

/* So sánh hai chuỗi hex cùng độ dài trong thời gian hằng (không thoát sớm
 * ở byte khác đầu tiên — không lộ độ dài tiền tố trùng qua thời gian). */
bool cmd_ct_equal(const char *a, const char *b, size_t n);

/* bytes -> hex thường. out phải có chỗ cho 2*n + 1. */
void cmd_hex_encode(const uint8_t *in, size_t n, char *out);

/* Viết chuỗi JSON (có ngoặc kép) vào out theo ĐÚNG cách Python json.dumps
 * với ensure_ascii=True: `"`/`\` và \b\f\n\r\t escape ngắn, ký tự điều
 * khiển khác -> \u00XX, UTF-8 2/3 byte -> \uXXXX (chữ thường, giống
 * Python). Chuỗi UTF-8 hỏng hoặc 4 byte -> '?' (vẫn nhất quán: edge đọc
 * lại đúng '?' và canonical ra cùng chuỗi đã ký).
 * Trả số byte đã viết (không tính NUL), hoặc -1 nếu không đủ chỗ. */
int cmd_json_escape(char *out, size_t cap, const char *s);

/* Viết thân ack canonical (CHƯA có sig):
 *   {"detail":"...","id":N,"ok":true|false[,"request_id":"..."]}
 * request_id NULL/"" thì bỏ. Trả độ dài, -1 nếu tràn. */
int cmd_ack_build(char *out, size_t cap, long id, bool ok,
                  const char *detail, const char *request_id);

/* Chèn `,"sig":"<hex>"` trước dấu `}` cuối của body (len = độ dài hiện
 * tại). Vị trí sig không quan trọng với bên xác minh — nó parse rồi
 * re-canonical. Trả độ dài mới, -1 nếu tràn hoặc body không kết thúc '}'. */
int cmd_ack_append_sig(char *body, size_t cap, size_t len, const char *sig_hex);

/* Cửa sổ chống chạy lại: nhớ CMD_DEDUP_N khoá gần nhất đã thực thi OK.
 *
 * Khoá = request_id nếu có (ổn định qua lần edge khởi động lại), không thì
 * id. Lưu băm FNV-1a 64 bit thay vì chuỗi để cửa sổ chỉ tốn 256 byte RAM
 * thay vì ~2 KB — xác suất đụng băm trong 32 khoá ~2^-59, chấp nhận. */
#define CMD_DEDUP_N 32

typedef struct {
    uint64_t key[CMD_DEDUP_N];
    uint8_t  used;   /* số ô đã dùng, tối đa CMD_DEDUP_N */
    uint8_t  next;   /* ô sẽ ghi đè kế tiếp (vòng tròn) */
} cmd_dedup_t;

uint64_t cmd_dedup_key(const char *request_id, long id);
bool     cmd_dedup_seen(const cmd_dedup_t *w, uint64_t key);
void     cmd_dedup_remember(cmd_dedup_t *w, uint64_t key);

/* Độ sâu lồng [ / { (ngoài chuỗi, bỏ qua escape) <= max_depth.
 *
 * Gọi TRƯỚC cJSON_Parse: cJSON đệ quy ~64 byte stack mỗi cấp, link_task chỉ
 * 3072 byte -> ~35 cấp '[' (gói ~40 byte, chưa cần ký) là tràn stack và
 * reboot node (đo -fstack-usage, review 2026-10-02). CONFIG_CJSON_NESTING_LIMIT
 * mặc định 1000 không cứu được. Lệnh hợp lệ chỉ sâu 1. */
bool cmd_json_depth_ok(const char *raw, size_t len, int max_depth);

/* Kiểm ts (unix GIÂY, giờ edge) so với giờ node. now_ms <= 0 nghĩa là
 * node chưa có giờ hợp lệ -> không chứng minh được còn mới -> false. */
bool cmd_ts_fresh(int64_t ts_s, int64_t now_ms, int64_t max_skew_s);
