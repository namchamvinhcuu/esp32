/* Host-side unit test for cmd_auth.c (no ESP-IDF needed).
 * Build & run (from esp32/ root):
 *   gcc -Wall -Wextra -I components/mqtt_link tests/host/test_cmd_auth.c \
 *       components/mqtt_link/cmd_auth.c -o /tmp/test_cmd_auth
 *   /tmp/test_cmd_auth
 *
 * Các vector CANONICAL/HMAC dưới đây sinh THẬT bằng Python, y như
 * edge_collector (manager.py::queue_command + mqtt_consumer.py::_sign):
 *   json.dumps(payload, sort_keys=True, separators=(",", ":"))
 *   hmac.new(b"k3y", canonical, hashlib.sha256).hexdigest()
 * Đổi format bên Python -> sinh lại vector, đừng sửa tay. Bắt được gói lệnh
 * THẬT từ edge (mosquitto_sub) -> thêm vào SIG_VECTORS trước khi tin parser. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmd_auth.h"

static int g_fail;

static void check(const char *name, int cond)
{
    printf("%-58s [%s]\n", name, cond ? "PASS" : "FAIL");
    if (!cond) {
        g_fail++;
    }
}

static void check_str(const char *name, const char *got, const char *exp)
{
    int pass = got != NULL && strcmp(got, exp) == 0;
    check(name, pass);
    if (!pass) {
        printf("    expected: %s\n    got     : %s\n", exp, got ? got : "(null)");
    }
}

/* ---------------------------------------------------------------- sig_strip */

typedef struct {
    const char *name;
    const char *raw;        /* gói edge publish (canonical CÓ sig) */
    const char *canonical;  /* canonical KHÔNG sig = thông điệp đã ký */
    const char *sig;        /* hmac-sha256("k3y", canonical) */
} sig_vec_t;

static const sig_vec_t SIG_VECTORS[] = {
    { "strip: request_id 32 hex + ms",
      "{\"channel\":\"relay1\",\"cmd\":\"write\",\"id\":7,\"ms\":500,\"request_id\":\"0123456789abcdef0123456789abcdef\",\"sig\":\"d5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1cc\",\"ts\":1790000000,\"value\":1}",
      "{\"channel\":\"relay1\",\"cmd\":\"write\",\"id\":7,\"ms\":500,\"request_id\":\"0123456789abcdef0123456789abcdef\",\"ts\":1790000000,\"value\":1}",
      "d5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1cc" },
    { "strip: pulse ms + period_ms + value true",
      "{\"channel\":\"tower\",\"cmd\":\"pulse\",\"id\":12,\"ms\":3000,\"period_ms\":250,\"sig\":\"15854a97bd072ea5103e3b8bbc2421bc6c924e8adcd2c496af0a7173d604ea8b\",\"ts\":1790000123,\"value\":true}",
      "{\"channel\":\"tower\",\"cmd\":\"pulse\",\"id\":12,\"ms\":3000,\"period_ms\":250,\"ts\":1790000123,\"value\":true}",
      "15854a97bd072ea5103e3b8bbc2421bc6c924e8adcd2c496af0a7173d604ea8b" },
    { "strip: value float 9.5",
      "{\"channel\":\"ao1\",\"cmd\":\"write\",\"id\":3,\"sig\":\"4cf230459d5fe7b64fcf511671315495fee098edf6ea7d3efc1a2b64c4e2e5d6\",\"ts\":1790000001,\"value\":9.5}",
      "{\"channel\":\"ao1\",\"cmd\":\"write\",\"id\":3,\"ts\":1790000001,\"value\":9.5}",
      "4cf230459d5fe7b64fcf511671315495fee098edf6ea7d3efc1a2b64c4e2e5d6" },
    { "strip: value null",
      "{\"channel\":\"relay2\",\"cmd\":\"read\",\"id\":4,\"sig\":\"932cce6b89ec0058cb118f866fd465343f8bfd8135932485e81f0de7532cfb1e\",\"ts\":1790000002,\"value\":null}",
      "{\"channel\":\"relay2\",\"cmd\":\"read\",\"id\":4,\"ts\":1790000002,\"value\":null}",
      "932cce6b89ec0058cb118f866fd465343f8bfd8135932485e81f0de7532cfb1e" },
    { "strip: channel non-ASCII (Python \\uXXXX + surrogate pair)",
      "{\"channel\":\"r\\u01a1le_\\u0111\\u00e8n\\u2014\\ud83d\\ude00\",\"cmd\":\"write\",\"id\":5,\"sig\":\"a515f8860084edbfc1a823d9202da90e27a625121903f1fbd9a0fe665b6a15a6\",\"ts\":1790000003,\"value\":0}",
      "{\"channel\":\"r\\u01a1le_\\u0111\\u00e8n\\u2014\\ud83d\\ude00\",\"cmd\":\"write\",\"id\":5,\"ts\":1790000003,\"value\":0}",
      "a515f8860084edbfc1a823d9202da90e27a625121903f1fbd9a0fe665b6a15a6" },
    /* channel = x,"sig":"<64 a>"  và value = \","sig":"  (Python escape) —
     * không được nhận nhầm là sig thật, cũng không tính là "2 sig". */
    { "strip: ,\"sig\":\" nam TRONG gia tri chuoi (escaped)",
      "{\"channel\":\"x,\\\"sig\\\":\\\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\\\"\",\"cmd\":\"write\",\"id\":6,\"sig\":\"3ad61fa0229b25648264e1dcf894f34f4482d03db8f98bcd3bf5ddac8f40f533\",\"ts\":1790000004,\"value\":\"\\\\\\\",\\\"sig\\\":\\\"\"}",
      "{\"channel\":\"x,\\\"sig\\\":\\\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\\\"\",\"cmd\":\"write\",\"id\":6,\"ts\":1790000004,\"value\":\"\\\\\\\",\\\"sig\\\":\\\"\"}",
      "3ad61fa0229b25648264e1dcf894f34f4482d03db8f98bcd3bf5ddac8f40f533" },
};

#define SIG64 "d5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1cc"

/* Gọi strip với out lót canary 'Z'; trả kết quả + để out/sig cho người gọi. */
static int strip(const char *raw, size_t cap, char *out, size_t *out_len,
                 char sig[CMD_SIG_HEX_LEN + 1])
{
    memset(out, 'Z', 512);
    memset(sig, 'Z', CMD_SIG_HEX_LEN + 1);
    *out_len = 9999;
    return cmd_sig_strip(raw, strlen(raw), out, cap, out_len, sig);
}

static int canary_ok(const char *buf, size_t from, size_t to)
{
    for (size_t i = from; i < to; i++) {
        if (buf[i] != 'Z') {
            return 0;
        }
    }
    return 1;
}

static void test_sig_strip(void)
{
    char out[512];
    char sig[CMD_SIG_HEX_LEN + 1];
    size_t out_len;
    char name[128];

    for (size_t i = 0; i < sizeof(SIG_VECTORS) / sizeof(SIG_VECTORS[0]); i++) {
        const sig_vec_t *v = &SIG_VECTORS[i];
        int ok = strip(v->raw, sizeof(out), out, &out_len, sig);
        snprintf(name, sizeof(name), "%s -> ok", v->name);
        check(name, ok);
        snprintf(name, sizeof(name), "%s -> canonical", v->name);
        check_str(name, ok ? out : NULL, v->canonical);
        snprintf(name, sizeof(name), "%s -> out_len", v->name);
        check(name, ok && out_len == strlen(v->canonical));
        snprintf(name, sizeof(name), "%s -> sig_hex", v->name);
        check_str(name, ok ? sig : NULL, v->sig);
    }

    /* Không có sig: canonical KHÔNG sig (kể cả loại có marker giả trong chuỗi) */
    check("strip: khong co sig -> false",
          !strip(SIG_VECTORS[0].canonical, sizeof(out), out, &out_len, sig));
    check("strip: chi co marker gia trong chuoi, khong sig that -> false",
          !strip(SIG_VECTORS[5].canonical, sizeof(out), out, &out_len, sig));
    check("strip: chuoi rong -> false",
          !strip("", sizeof(out), out, &out_len, sig));

    /* 2 sig */
    check("strip: hai sig -> false",
          !strip("{\"a\":1,\"sig\":\"" SIG64 "\",\"sig\":\"" SIG64 "\",\"ts\":1}",
                 sizeof(out), out, &out_len, sig));

    /* hình dạng sig sai */
    check("strip: sig 63 hex -> false",
          !strip("{\"a\":1,\"sig\":\"d5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1c\",\"ts\":1}",
                 sizeof(out), out, &out_len, sig));
    check("strip: sig 65 hex -> false",
          !strip("{\"a\":1,\"sig\":\"" SIG64 "0\",\"ts\":1}",
                 sizeof(out), out, &out_len, sig));
    check("strip: sig hex HOA -> false",
          !strip("{\"a\":1,\"sig\":\"D5993BDF17A73BD2CBFCDAB900B6E83B5AF1380E551E4A8A18FFEABAC773A1CC\",\"ts\":1}",
                 sizeof(out), out, &out_len, sig));
    check("strip: sig co ky tu 'g' -> false",
          !strip("{\"a\":1,\"sig\":\"g5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1cc\",\"ts\":1}",
                 sizeof(out), out, &out_len, sig));
    check("strip: thieu dau \" dong (con byte khac) -> false",
          !strip("{\"a\":1,\"sig\":\"" SIG64 ",\"ts\":1}",
                 sizeof(out), out, &out_len, sig));
    check("strip: raw het ngay sau 64 hex (thieu \" dong) -> false",
          !strip("{\"a\":1,\"sig\":\"" SIG64, sizeof(out), out, &out_len, sig));
    check("strip: raw cut giua hex -> false",
          !strip("{\"a\":1,\"sig\":\"d5993bdf", sizeof(out), out, &out_len, sig));
    /* sig là key đầu tiên (không có ',' trước) — không xảy ra với payload
     * thật (channel/cmd/id luôn sort trước "sig") nên từ chối là đúng. */
    check("strip: sig la key dau tien (khong co ',' truoc) -> false",
          !strip("{\"sig\":\"" SIG64 "\",\"ts\":1}", sizeof(out), out, &out_len, sig));

    /* raw_len giới hạn vùng quét: sig nằm ngoài raw_len không được thấy */
    {
        const char *raw = SIG_VECTORS[2].raw;
        const char *m = strstr(raw, ",\"sig\"");
        memset(out, 'Z', sizeof(out));
        check("strip: sig nam ngoai raw_len -> false",
              !cmd_sig_strip(raw, (size_t)(m - raw) + 5, out, sizeof(out), &out_len, sig));
    }

    /* out_cap: vừa đủ (len+1) OK; len -> false, không ghi gì */
    {
        const sig_vec_t *v = &SIG_VECTORS[2];
        size_t need = strlen(v->canonical);
        int ok = strip(v->raw, need + 1, out, &out_len, sig);
        check("strip: out_cap = len+1 (vua du) -> ok", ok && out_len == need);
        check_str("strip: out_cap = len+1 -> canonical", ok ? out : NULL, v->canonical);
        check("strip: out_cap = len+1 -> khong ghi vuot cap", canary_ok(out, need + 1, sizeof(out)));

        ok = strip(v->raw, need, out, &out_len, sig);
        check("strip: out_cap = len (thieu 1 cho NUL) -> false", !ok);
        check("strip: out_cap qua nho -> out khong bi ghi", canary_ok(out, 0, sizeof(out)));
        check("strip: out_cap qua nho -> out_len khong doi", out_len == 9999);

        check("strip: out_cap = 0 -> false", !strip(v->raw, 0, out, &out_len, sig));
    }
}

/* ------------------------------------------------- hex_encode / ct_equal */

static void test_hex_ct(void)
{
    const uint8_t in[] = { 0x00, 0x0f, 0xa5, 0xff, 0x10 };
    char out[2 * sizeof(in) + 1];
    cmd_hex_encode(in, sizeof(in), out);
    check_str("hex_encode: chu thuong, dung thu tu nibble", out, "000fa5ff10");
    cmd_hex_encode(in, 0, out);
    check_str("hex_encode: n=0 -> chuoi rong", out, "");

    check("ct_equal: bang nhau -> true", cmd_ct_equal(SIG64, SIG64, 64));
    check("ct_equal: khac byte dau -> false",
          !cmd_ct_equal(SIG64, "e5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1cc", 64));
    check("ct_equal: khac byte cuoi -> false",
          !cmd_ct_equal(SIG64, "d5993bdf17a73bd2cbfcdab900b6e83b5af1380e551e4a8a18ffeabac773a1cd", 64));
}

/* ------------------------------------------------------------ json_escape */

static void check_esc(const char *name, const char *in, const char *exp)
{
    char out[256];
    int n = cmd_json_escape(out, sizeof(out), in);
    check_str(name, n >= 0 ? out : NULL, exp);
    if (n >= 0 && (size_t)n != strlen(exp)) {
        check("    (gia tri tra ve == strlen)", 0);
    }
}

static void test_json_escape(void)
{
    /* Kỳ vọng = Python json.dumps(s) (ensure_ascii=True mặc định). */
    check_esc("escape: ASCII thuong", "relay set", "\"relay set\"");
    check_esc("escape: chuoi rong", "", "\"\"");
    check_esc("escape: NULL -> \"\"", NULL, "\"\"");
    check_esc("escape: \" va \\", "a\"b\\c", "\"a\\\"b\\\\c\"");
    check_esc("escape: \\n \\r \\t \\b \\f", "\n\r\t\b\f", "\"\\n\\r\\t\\b\\f\"");
    check_esc("escape: 0x01 -> \\u0001", "\x01", "\"\\u0001\"");
    check_esc("escape: 0x1f -> \\u001f", "\x1f", "\"\\u001f\"");
    check_esc("escape: 0x7f -> \\u007f", "\x7f", "\"\\u007f\"");
    check_esc("escape: ' ' va '~' giu nguyen", " ~", "\" ~\"");
    check_esc("escape: e-acute (2 byte) -> \\u00e9", "\xc3\xa9", "\"\\u00e9\"");
    check_esc("escape: em-dash (3 byte) -> \\u2014", "\xe2\x80\x94", "\"\\u2014\"");
    check_esc("escape: U+0080 (2 byte nho nhat)", "\xc2\x80", "\"\\u0080\"");
    check_esc("escape: U+FFFF (3 byte lon nhat)", "\xef\xbf\xbf", "\"\\uffff\"");
    check_esc("escape: emoji 4 byte -> mot '?'", "x\xf0\x9f\x98\x80y", "\"x?y\"");
    check_esc("escape: UTF-8 hong: c3 + '(' -> ?(", "\xc3(", "\"?(\"");
    check_esc("escape: UTF-8 hong: 3 byte bi cut cuoi chuoi", "a\xe2\x80", "\"a?\"");
    check_esc("escape: UTF-8 hong: ff fe -> ??", "\xff\xfe", "\"??\"");
    check_esc("escape: overlong c0 af -> ?", "\xc0\xaf", "\"?\"");
    check_esc("escape: overlong 3 byte e0 80 af -> ?", "\xe0\x80\xaf", "\"?\"");
    check_esc("escape: surrogate ed a0 80 -> ?", "\xed\xa0\x80", "\"?\"");
    check_esc("escape: continuation le 80 -> ?", "a\x80" "b", "\"a?b\"");

    /* Tràn buffer: "ab" -> "\"ab\"" cần 4 + NUL = 5 */
    {
        char out[16];
        memset(out, 'Z', sizeof(out));
        check("escape: cap = 5 (vua du) -> 4", cmd_json_escape(out, 5, "ab") == 4);
        check_str("escape: cap = 5 -> noi dung", out, "\"ab\"");
        check("escape: cap = 5 -> khong ghi vuot cap", canary_ok(out, 5, sizeof(out)));
        for (size_t cap = 0; cap < 5; cap++) {
            memset(out, 'Z', sizeof(out));
            int n = cmd_json_escape(out, cap, "ab");
            char name[96];
            snprintf(name, sizeof(name), "escape: cap = %zu -> -1, khong ghi vuot cap", cap);
            check(name, n == -1 && canary_ok(out, cap, sizeof(out)));
        }
        /* escape 6 byte \u2014 không được cắt giữa chừng: "\"\\u2014\"" = 8 */
        memset(out, 'Z', sizeof(out));
        check("escape: cap = 8 cho \\u2014 (thieu NUL) -> -1",
              cmd_json_escape(out, 8, "\xe2\x80\x94") == -1 && canary_ok(out, 8, sizeof(out)));
        check("escape: cap = 9 cho \\u2014 -> 8", cmd_json_escape(out, 9, "\xe2\x80\x94") == 8);
    }
}

/* ------------------------------------------------- ack_build / append_sig */

static void test_ack(void)
{
    char out[256];
    int n;

    /* Kỳ vọng = Python json.dumps({...}, sort_keys=True, separators=(",",":")) */
    n = cmd_ack_build(out, sizeof(out), 7, true, "ok", NULL);
    check_str("ack: co ban, request_id NULL", n >= 0 ? out : NULL,
              "{\"detail\":\"ok\",\"id\":7,\"ok\":true}");
    check("ack: gia tri tra ve == strlen", n == (int)strlen(out));

    n = cmd_ack_build(out, sizeof(out), 7, true, "ok", "");
    check_str("ack: request_id \"\" -> bo field", n >= 0 ? out : NULL,
              "{\"detail\":\"ok\",\"id\":7,\"ok\":true}");

    n = cmd_ack_build(out, sizeof(out), 12, false, "relay set",
                      "0123456789abcdef0123456789abcdef");
    check_str("ack: ok=false + request_id", n >= 0 ? out : NULL,
              "{\"detail\":\"relay set\",\"id\":12,\"ok\":false,\"request_id\":\"0123456789abcdef0123456789abcdef\"}");

    n = cmd_ack_build(out, sizeof(out), -1, true,
                      "a\"b\\c\nd\re\tf\bg\fh\x01i\x7fj\xe2\x80\x94k\xc3\xa9l", NULL);
    check_str("ack: detail day du ky tu dac biet (vector Python)", n >= 0 ? out : NULL,
              "{\"detail\":\"a\\\"b\\\\c\\nd\\re\\tf\\bg\\fh\\u0001i\\u007fj\\u2014k\\u00e9l\",\"id\":-1,\"ok\":true}");

    n = cmd_ack_build(out, sizeof(out), 0, false, "", NULL);
    check_str("ack: detail rong, id 0", n >= 0 ? out : NULL,
              "{\"detail\":\"\",\"id\":0,\"ok\":false}");

    n = cmd_ack_build(out, sizeof(out), 1, true, NULL, NULL);
    check_str("ack: detail NULL -> \"\"", n >= 0 ? out : NULL,
              "{\"detail\":\"\",\"id\":1,\"ok\":true}");

    /* Tràn: quét mọi cap < cần -> -1 và không ghi vượt cap */
    {
        const char *rid = "0123456789abcdef0123456789abcdef";
        char ref[256];
        int need = cmd_ack_build(ref, sizeof(ref), 12, false, "relay \xe2\x80\x94 set", rid);
        int all_neg = 1, all_canary = 1;
        for (int cap = 0; cap <= need; cap++) {
            memset(out, 'Z', sizeof(out));
            int k = cmd_ack_build(out, (size_t)cap, 12, false, "relay \xe2\x80\x94 set", rid);
            if (k != -1) {
                all_neg = 0;
                printf("    cap=%d -> %d (mong -1)\n", cap, k);
            }
            if (!canary_ok(out, (size_t)cap, sizeof(out))) {
                all_canary = 0;
                printf("    cap=%d -> ghi vuot cap\n", cap);
            }
        }
        check("ack: moi cap 0..len -> -1", need > 0 && all_neg);
        check("ack: moi cap 0..len -> khong ghi vuot cap", all_canary);
        memset(out, 'Z', sizeof(out));
        n = cmd_ack_build(out, (size_t)need + 1, 12, false, "relay \xe2\x80\x94 set", rid);
        check("ack: cap = len+1 -> ok", n == need && strcmp(out, ref) == 0);
    }

    /* append_sig */
    {
        const char *body = "{\"detail\":\"ok\",\"id\":7,\"ok\":true}";
        const char *exp = "{\"detail\":\"ok\",\"id\":7,\"ok\":true,\"sig\":\"" SIG64 "\"}";
        size_t blen = strlen(body), elen = strlen(exp);

        strcpy(out, body);
        n = cmd_ack_append_sig(out, sizeof(out), blen, SIG64);
        check_str("append_sig: chen truoc '}' cuoi", n >= 0 ? out : NULL, exp);
        check("append_sig: tra do dai moi", n == (int)elen);

        memset(out, 'Z', sizeof(out));
        strcpy(out, body);
        n = cmd_ack_append_sig(out, elen + 1, blen, SIG64);
        check("append_sig: cap = len+1 (vua du) -> ok", n == (int)elen && strcmp(out, exp) == 0);

        memset(out, 'Z', sizeof(out));
        strcpy(out, body);
        n = cmd_ack_append_sig(out, elen, blen, SIG64);
        check("append_sig: cap = len (thieu NUL) -> -1", n == -1);
        check_str("append_sig: tran -> body tra ve nguyen trang", out, body);
        check("append_sig: tran -> khong ghi vuot cap", canary_ok(out, elen, sizeof(out)));

        strcpy(out, "{\"a\":1");
        check("append_sig: body khong ket thuc '}' -> -1",
              cmd_ack_append_sig(out, sizeof(out), strlen(out), SIG64) == -1);
        check("append_sig: len = 0 -> -1", cmd_ack_append_sig(out, sizeof(out), 0, SIG64) == -1);

        /* round-trip: build + append + strip = đúng body đã ký */
        char built[256], back[256], sig[CMD_SIG_HEX_LEN + 1];
        size_t back_len;
        int bl = cmd_ack_build(built, sizeof(built), 12, false, "relay set",
                               "0123456789abcdef0123456789abcdef");
        char signed_ack[256];
        strcpy(signed_ack, built);
        int sl = cmd_ack_append_sig(signed_ack, sizeof(signed_ack), (size_t)bl, SIG64);
        int ok = sl > 0 && cmd_sig_strip(signed_ack, (size_t)sl, back, sizeof(back),
                                         &back_len, sig);
        check("ack round-trip: strip(append(build)) == build", ok && strcmp(back, built) == 0);
        check_str("ack round-trip: sig giu nguyen", ok ? sig : NULL, SIG64);
    }
}

/* ------------------------------------------------------------------ dedup */

static void test_dedup(void)
{
    cmd_dedup_t w;
    memset(&w, 0, sizeof(w));
    const uint64_t A = cmd_dedup_key("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 1);
    const uint64_t B = cmd_dedup_key("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", 1);

    check("dedup: cua so rong -> A chua seen", !cmd_dedup_seen(&w, A));
    cmd_dedup_remember(&w, A);
    check("dedup: A,B,A -> B lan dau chua seen", !cmd_dedup_seen(&w, B));
    cmd_dedup_remember(&w, B);
    check("dedup: A,B,A -> A lan 2 seen", cmd_dedup_seen(&w, A));

    check("dedup: request_id \"123\" khac id 123",
          cmd_dedup_key("123", 0) != cmd_dedup_key(NULL, 123));
    check("dedup: request_id \"\" -> dung id (bang key NULL)",
          cmd_dedup_key("", 42) == cmd_dedup_key(NULL, 42));
    check("dedup: co request_id thi bo qua id",
          cmd_dedup_key("abc", 1) == cmd_dedup_key("abc", 2));
    check("dedup: id khac nhau -> key khac",
          cmd_dedup_key(NULL, 1) != cmd_dedup_key(NULL, 2));
    check("dedup: id am / LONG_MAX khong trung",
          cmd_dedup_key(NULL, -1) != cmd_dedup_key(NULL, 2147483647L));

    /* remember trùng không chiếm ô */
    memset(&w, 0, sizeof(w));
    cmd_dedup_remember(&w, A);
    cmd_dedup_remember(&w, A);
    check("dedup: remember trung -> used == 1, next == 1", w.used == 1 && w.next == 1);

    /* A, B, B, rồi 30 khoá khác -> đúng 32 khoá khác nhau, A phải còn */
    memset(&w, 0, sizeof(w));
    cmd_dedup_remember(&w, A);
    cmd_dedup_remember(&w, B);
    cmd_dedup_remember(&w, B);
    for (long i = 0; i < 30; i++) {
        cmd_dedup_remember(&w, cmd_dedup_key(NULL, 1000 + i));
    }
    check("dedup: A,B,B + 30 khac -> A van seen (trung khong chiem o)",
          cmd_dedup_seen(&w, A));
    check("dedup: day 32 o -> used == 32", w.used == CMD_DEDUP_N);

    /* khoá thứ 33 đẩy khoá cũ nhất (A) ra, các khoá khác còn */
    cmd_dedup_remember(&w, cmd_dedup_key(NULL, 5000));
    check("dedup: khoa thu 33 -> A (cu nhat) bi day ra", !cmd_dedup_seen(&w, A));
    check("dedup: khoa thu 33 -> B (cu nhi) van seen", cmd_dedup_seen(&w, B));
    check("dedup: khoa thu 33 -> khoa moi seen", cmd_dedup_seen(&w, cmd_dedup_key(NULL, 5000)));
    check("dedup: used giu 32 sau khi day", w.used == CMD_DEDUP_N);
    cmd_dedup_remember(&w, cmd_dedup_key(NULL, 5001));
    check("dedup: khoa thu 34 -> B bi day ra", !cmd_dedup_seen(&w, B));
    check("dedup: khoa thu 34 -> khoa 1000 (thu 3) van seen",
          cmd_dedup_seen(&w, cmd_dedup_key(NULL, 1000)));

    /* chạy vòng nhiều lần: chỉ 32 khoá gần nhất còn */
    memset(&w, 0, sizeof(w));
    for (long i = 0; i < 100; i++) {
        cmd_dedup_remember(&w, cmd_dedup_key(NULL, i));
    }
    int recent_ok = 1;
    for (long i = 68; i < 100; i++) {
        recent_ok &= cmd_dedup_seen(&w, cmd_dedup_key(NULL, i));
    }
    check("dedup: 100 khoa -> 32 khoa gan nhat (68..99) seen", recent_ok);
    check("dedup: 100 khoa -> khoa 67 da bi day ra", !cmd_dedup_seen(&w, cmd_dedup_key(NULL, 67)));
}

/* -------------------------------------------------------- json_depth_ok */

static int depth_ok(const char *s, int max) { return cmd_json_depth_ok(s, strlen(s), max); }

static void test_json_depth(void)
{
    /* Regression P1 (review vòng 2, 2026-10-02): JSON lồng sâu làm tràn stack
     * link_task trong cJSON_Parse TRƯỚC bước auth. mqtt_link.c gọi max=2. */
    check("depth: [[[ voi max 2 -> false", !depth_ok("[[[", 2));
    check("depth: {\"a\":{\"a\":{ voi max 2 -> false", !depth_ok("{\"a\":{\"a\":{", 2));
    check("depth: {\"a\":{\"a\":1}} (sau 2) voi max 2 -> true", depth_ok("{\"a\":{\"a\":1}}", 2));
    check("depth: [[ dung bang max 2 -> true", depth_ok("[[", 2));
    check("depth: tron [{[ voi max 2 -> false", !depth_ok("[{\"a\":[1]}]", 2));
    {
        char deep[200];
        memset(deep, '[', sizeof(deep) - 1);
        deep[sizeof(deep) - 1] = '\0';
        check("depth: 199 x '[' (payload tan cong) -> false", !depth_ok(deep, 2));
    }

    /* gói lệnh thật (sâu 1) */
    for (size_t i = 0; i < sizeof(SIG_VECTORS) / sizeof(SIG_VECTORS[0]); i++) {
        char name[128];
        snprintf(name, sizeof(name), "depth: goi that #%zu (sau 1) max 2 -> true", i);
        check(name, depth_ok(SIG_VECTORS[i].raw, 2));
    }
    check("depth: goi that sau 1 voi max 1 -> true", depth_ok(SIG_VECTORS[0].raw, 1));
    check("depth: goi that sau 1 voi max 0 -> false", !depth_ok(SIG_VECTORS[0].raw, 0));

    /* [ { trong chuỗi không đếm */
    check("depth: [[[ va {{{ trong chuoi -> true",
          depth_ok("{\"channel\":\"[[[{{{[[[\",\"v\":\"{{{{\"}", 1));
    check("depth: escaped \\\" giu nguyen trong chuoi, [[[ sau no khong dem",
          depth_ok("{\"a\":\"x\\\"[[[\\\"{{{\"}", 1));
    /* chuỗi kết thúc bằng backslash escape: "a\\" đóng chuỗi, [[[ sau phải đếm */
    check("depth: \"a\\\\\" roi [[[ -> false (backslash escape dong chuoi)",
          !depth_ok("{\"a\":\"a\\\\\",\"b\":[[[1]]]}", 2));
    check("depth: \"a\\\\\" roi [1] -> true",
          depth_ok("{\"a\":\"a\\\\\",\"b\":[1]}", 2));
    check("depth: \"\\\\\\\"[[[\" (\\\\ roi \\\") van trong chuoi -> true",
          depth_ok("{\"a\":\"\\\\\\\"[[[\"}", 1));

    /* len */
    check("depth: len = 0 -> true", cmd_json_depth_ok("[[[[", 0, 2));
    check("depth: chi xet trong len ({} roi [[[ ngoai len) -> true",
          cmd_json_depth_ok("{}[[[", 2, 2));
    check("depth: [[[ cat len = 2 -> true", cmd_json_depth_ok("[[[", 2, 2));
    check("depth: [[[ len = 3 -> false", !cmd_json_depth_ok("[[[", 3, 2));
    check("depth: backslash la byte cuoi trong len -> true, khong doc vuot",
          cmd_json_depth_ok("\"ab\\[[[[", 4, 2));
    /* đóng mở nối tiếp không cộng dồn */
    check("depth: [][][][] (moi cap sau 1) max 1 -> true", depth_ok("[][][][]", 1));

    /* Vòng 3: đóng nhiều hơn mở (depth âm) phải bị từ chối, không được để
     * phần mở phía sau "bù" lại độ sâu âm. */
    check("depth: ]]][[[ -> false (dong truoc mo)", !depth_ok("]]][[[", 2));
    check("depth: {}]]]]]][[[[[[ -> false", !depth_ok("{}]]]]]][[[[[[", 2));
    check("depth: } le -> false", !depth_ok("}", 2));
    check("depth: {}] -> false (du 1 dau dong)", !depth_ok("{}]", 2));
    check("depth: ] trong chuoi khong dem -> true", depth_ok("{\"a\":\"]]]\"}", 1));
    check("depth: {} voi max 1 -> true", depth_ok("{}", 1));
    check("depth: [][][] voi max 1 -> true", depth_ok("[][][]", 1));
}

/* ----------------------------------------------------------------- ts_fresh */

static void test_ts_fresh(void)
{
    const int64_t ts = 1790000000;
    const int64_t now = ts * 1000;

    check("ts: now_ms = 0 -> false", !cmd_ts_fresh(ts, 0, 120));
    check("ts: now_ms < 0 -> false", !cmd_ts_fresh(ts, -5, 120));
    check("ts: now_ms = 0, ts = 0 -> false (chua co gio)", !cmd_ts_fresh(0, 0, 120));
    check("ts: lech 0 -> true", cmd_ts_fresh(ts, now, 120));
    check("ts: ts cu dung 120s -> true", cmd_ts_fresh(ts, now + 120000, 120));
    check("ts: ts cu 121s -> false", !cmd_ts_fresh(ts, now + 121000, 120));
    check("ts: ts cu 120.999s (ms bi cat ve giay) -> true", cmd_ts_fresh(ts, now + 120999, 120));
    check("ts: ts tuong lai 120s -> true", cmd_ts_fresh(ts + 120, now, 120));
    check("ts: ts tuong lai 121s -> false", !cmd_ts_fresh(ts + 121, now, 120));
    check("ts: ts = 0 (thieu ts) voi gio that -> false", !cmd_ts_fresh(0, now, 120));
}

int main(void)
{
    test_sig_strip();
    test_hex_ct();
    test_json_escape();
    test_ack();
    test_dedup();
    test_json_depth();
    test_ts_fresh();

    if (g_fail) {
        printf("\n%d FAILURE(S)\n", g_fail);
        return 1;
    }
    printf("\nALL PASS\n");
    return 0;
}
