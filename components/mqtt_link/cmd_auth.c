#include "cmd_auth.h"

#include <stdio.h>
#include <string.h>

static const char SIG_MARK[] = ",\"sig\":\"";
#define SIG_MARK_LEN (sizeof(SIG_MARK) - 1)

static bool is_lower_hex(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

/* memmem không có trong mọi libc nhúng — tự viết, chuỗi chỉ vài trăm byte. */
static const char *find_mark(const char *p, const char *end)
{
    for (; p + SIG_MARK_LEN <= end; p++) {
        if (memcmp(p, SIG_MARK, SIG_MARK_LEN) == 0) {
            return p;
        }
    }
    return NULL;
}

bool cmd_sig_strip(const char *raw, size_t raw_len,
                   char *out, size_t out_cap, size_t *out_len,
                   char sig_hex[CMD_SIG_HEX_LEN + 1])
{
    const char *end = raw + raw_len;
    const char *m = find_mark(raw, end);
    if (m == NULL) {
        return false;
    }
    if (find_mark(m + 1, end) != NULL) {
        return false;                    /* hai sig: không biết cái nào được ký */
    }
    const char *h = m + SIG_MARK_LEN;
    if (h + CMD_SIG_HEX_LEN + 1 > end) {
        return false;
    }
    for (size_t i = 0; i < CMD_SIG_HEX_LEN; i++) {
        if (!is_lower_hex(h[i])) {
            return false;
        }
    }
    if (h[CMD_SIG_HEX_LEN] != '"') {
        return false;
    }
    const char *tail = h + CMD_SIG_HEX_LEN + 1;
    const size_t head_len = (size_t)(m - raw);
    const size_t tail_len = (size_t)(end - tail);
    if (head_len + tail_len + 1 > out_cap) {
        return false;
    }
    memcpy(out, raw, head_len);
    memcpy(out + head_len, tail, tail_len);
    out[head_len + tail_len] = '\0';
    *out_len = head_len + tail_len;
    memcpy(sig_hex, h, CMD_SIG_HEX_LEN);
    sig_hex[CMD_SIG_HEX_LEN] = '\0';
    return true;
}

bool cmd_ct_equal(const char *a, const char *b, size_t n)
{
    unsigned char d = 0;
    for (size_t i = 0; i < n; i++) {
        d |= (unsigned char)(a[i] ^ b[i]);
    }
    return d == 0;
}

void cmd_hex_encode(const uint8_t *in, size_t n, char *out)
{
    static const char HEX[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[2 * i]     = HEX[in[i] >> 4];
        out[2 * i + 1] = HEX[in[i] & 0x0f];
    }
    out[2 * n] = '\0';
}

/* Giải mã một ký tự UTF-8 2 hoặc 3 byte (BMP). Trả số byte tiêu thụ, 0 nếu
 * hỏng / 4 byte / overlong / surrogate — người gọi thay bằng '?'. */
static int utf8_bmp(const unsigned char *s, unsigned *cp)
{
    if ((s[0] & 0xe0) == 0xc0) {
        if ((s[1] & 0xc0) != 0x80) {
            return 0;
        }
        *cp = ((unsigned)(s[0] & 0x1f) << 6) | (s[1] & 0x3f);
        return *cp >= 0x80 ? 2 : 0;
    }
    if ((s[0] & 0xf0) == 0xe0) {
        if ((s[1] & 0xc0) != 0x80 || (s[2] & 0xc0) != 0x80) {
            return 0;
        }
        *cp = ((unsigned)(s[0] & 0x0f) << 12) | ((unsigned)(s[1] & 0x3f) << 6) |
              (s[2] & 0x3f);
        if (*cp < 0x800 || (*cp >= 0xd800 && *cp <= 0xdfff)) {
            return 0;
        }
        return 3;
    }
    return 0;
}

int cmd_json_escape(char *out, size_t cap, const char *s)
{
    size_t len = 0;
#define PUT(str, n)                                  \
    do {                                             \
        if (len + (n) >= cap) {                      \
            return -1;                               \
        }                                            \
        memcpy(out + len, (str), (n));               \
        len += (n);                                  \
    } while (0)

    PUT("\"", 1);
    const unsigned char *p = (const unsigned char *)(s ? s : "");
    while (*p) {
        const unsigned char c = *p;
        char esc[8];
        if (c == '"') {
            PUT("\\\"", 2);
            p++;
        } else if (c == '\\') {
            PUT("\\\\", 2);
            p++;
        } else if (c == '\n') {
            PUT("\\n", 2);
            p++;
        } else if (c == '\r') {
            PUT("\\r", 2);
            p++;
        } else if (c == '\t') {
            PUT("\\t", 2);
            p++;
        } else if (c == '\b') {
            PUT("\\b", 2);
            p++;
        } else if (c == '\f') {
            PUT("\\f", 2);
            p++;
        } else if (c < 0x20 || c == 0x7f) {
            /* Python ESCAPE_ASCII: mọi thứ ngoài ' '..'~' -> \u00xx */
            snprintf(esc, sizeof(esc), "\\u%04x", (unsigned)c);
            PUT(esc, 6);
            p++;
        } else if (c < 0x80) {
            PUT((const char *)p, 1);
            p++;
        } else {
            unsigned cp = 0;
            const int n = utf8_bmp(p, &cp);
            if (n == 0) {
                PUT("?", 1);
                /* bỏ qua cả chuỗi byte tiếp nối để không sinh nhiều '?' */
                p++;
                while ((*p & 0xc0) == 0x80) {
                    p++;
                }
            } else {
                snprintf(esc, sizeof(esc), "\\u%04x", cp);
                PUT(esc, 6);
                p += n;
            }
        }
    }
    PUT("\"", 1);
#undef PUT
    out[len] = '\0';
    return (int)len;
}

int cmd_ack_build(char *out, size_t cap, long id, bool ok,
                  const char *detail, const char *request_id)
{
    int w = snprintf(out, cap, "{\"detail\":");
    if (w < 0 || (size_t)w >= cap) {
        return -1;
    }
    size_t len = (size_t)w;

    int k = cmd_json_escape(out + len, cap - len, detail);
    if (k < 0) {
        return -1;
    }
    len += (size_t)k;

    k = snprintf(out + len, cap - len, ",\"id\":%ld,\"ok\":%s", id,
                 ok ? "true" : "false");
    if (k < 0 || (size_t)k >= cap - len) {
        return -1;
    }
    len += (size_t)k;

    if (request_id != NULL && request_id[0] != '\0') {
        k = snprintf(out + len, cap - len, ",\"request_id\":");
        if (k < 0 || (size_t)k >= cap - len) {
            return -1;
        }
        len += (size_t)k;
        k = cmd_json_escape(out + len, cap - len, request_id);
        if (k < 0) {
            return -1;
        }
        len += (size_t)k;
    }

    if (len + 2 > cap) {
        return -1;
    }
    out[len++] = '}';
    out[len] = '\0';
    return (int)len;
}

int cmd_ack_append_sig(char *body, size_t cap, size_t len, const char *sig_hex)
{
    if (len == 0 || body[len - 1] != '}') {
        return -1;
    }
    int k = snprintf(body + len - 1, cap - (len - 1), ",\"sig\":\"%s\"}", sig_hex);
    if (k < 0 || (size_t)k >= cap - (len - 1)) {
        body[len - 1] = '}';   /* tràn: trả body về như cũ */
        body[len] = '\0';
        return -1;
    }
    return (int)(len - 1 + (size_t)k);
}

static uint64_t fnv1a(uint64_t h, const void *data, size_t n)
{
    const unsigned char *p = data;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 0x100000001b3ULL;
    }
    return h;
}

uint64_t cmd_dedup_key(const char *request_id, long id)
{
    uint64_t h = 0xcbf29ce484222325ULL;
    /* Tiền tố tách hai không gian khoá: request_id "123" khác id 123. */
    if (request_id != NULL && request_id[0] != '\0') {
        h = fnv1a(h, "r", 1);
        return fnv1a(h, request_id, strlen(request_id));
    }
    char buf[24];
    int n = snprintf(buf, sizeof(buf), "%ld", id);
    h = fnv1a(h, "i", 1);
    return fnv1a(h, buf, (n > 0 && (size_t)n < sizeof(buf)) ? (size_t)n : 0);
}

bool cmd_dedup_seen(const cmd_dedup_t *w, uint64_t key)
{
    for (unsigned i = 0; i < w->used; i++) {
        if (w->key[i] == key) {
            return true;
        }
    }
    return false;
}

void cmd_dedup_remember(cmd_dedup_t *w, uint64_t key)
{
    if (cmd_dedup_seen(w, key)) {
        return;
    }
    w->key[w->next] = key;
    w->next = (uint8_t)((w->next + 1) % CMD_DEDUP_N);
    if (w->used < CMD_DEDUP_N) {
        w->used++;
    }
}

bool cmd_json_depth_ok(const char *raw, size_t len, int max_depth)
{
    int depth = 0;
    bool in_str = false;
    for (size_t i = 0; i < len; i++) {
        const char c = raw[i];
        if (in_str) {
            if (c == '\\') {
                i++;            /* bỏ qua ký tự bị escape */
            } else if (c == '"') {
                in_str = false;
            }
        } else if (c == '"') {
            in_str = true;
        } else if (c == '[' || c == '{') {
            if (++depth > max_depth) {
                return false;
            }
        } else if (c == ']' || c == '}') {
            if (--depth < 0) {
                return false;   /* đóng nhiều hơn mở: không phải JSON hợp lệ */
            }
        }
    }
    return true;
}

bool cmd_ts_fresh(int64_t ts_s, int64_t now_ms, int64_t max_skew_s)
{
    if (now_ms <= 0) {
        return false;
    }
    int64_t d = now_ms / 1000 - ts_s;
    if (d < 0) {
        d = -d;
    }
    return d <= max_skew_s;
}

bool cmd_duration_ms(bool present, bool is_number, double v, int32_t *out)
{
    *out = 0;
    if (!present) {
        return true;
    }
    if (!is_number || v != v) {   /* v != v: NaN */
        return false;
    }
    if (v <= 0) {
        return true;
    }
    if (v >= 2147483647.0) {
        *out = INT32_MAX;
        return true;
    }
    *out = (int32_t)v;
    return true;
}
