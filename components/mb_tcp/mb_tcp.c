#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "cfg.h"
#include "meas_core.h"
#include "mb_tcp.h"

#define CONNECT_TIMEOUT_MS 2000
/* 1000ms: gateway TCP->RTU phai hoi lai cam bien phia sau moi tra loi —
 * 500ms cu lam node timeout oan khi gateway ban (churn connected ~70s/lan,
 * quan sat that 16/07). Doc cham hon 1s that su thi coi la chet. */
#define REQUEST_TIMEOUT_MS 1000
#define HOST_BACKOFF_MS    10000
#define MAX_HOSTS          4
#define MBAP_LEN           7

static const char *TAG = "mbtcp";

/* one cached connection per distinct host:port */
typedef struct {
    char     host[32];
    uint16_t port;
    int      sock;        /* -1 when closed */
    int64_t  blocked_until_us;
} mb_host_t;

static mb_host_t s_hosts[MAX_HOSTS];
static size_t    s_host_count;
static uint16_t  s_tid;

static mb_host_t *host_for(const cfg_channel_t *c)
{
    for (size_t i = 0; i < s_host_count; i++) {
        if (s_hosts[i].port == c->port &&
            strcmp(s_hosts[i].host, c->host) == 0) {
            return &s_hosts[i];
        }
    }
    if (s_host_count == MAX_HOSTS) {
        return NULL;
    }
    mb_host_t *h = &s_hosts[s_host_count++];
    strlcpy(h->host, c->host, sizeof(h->host));
    h->port = c->port;
    h->sock = -1;
    return h;
}

static void host_close(mb_host_t *h)
{
    if (h->sock >= 0) {
        close(h->sock);
        h->sock = -1;
    }
}

/* non-blocking connect with select() timeout, then SO_RCVTIMEO for requests */
static bool host_connect(mb_host_t *h)
{
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%u", (unsigned)h->port);
    if (getaddrinfo(h->host, port_str, &hints, &res) != 0 || res == NULL) {
        ESP_LOGW(TAG, "%s: dns/addr failed", h->host);
        return false;
    }

    int s = socket(res->ai_family, res->ai_socktype, 0);
    if (s < 0) {
        freeaddrinfo(res);
        return false;
    }

    int fl = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, fl | O_NONBLOCK);

    int rc = connect(s, res->ai_addr, res->ai_addrlen);
    if (rc < 0 && errno == EINPROGRESS) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(s, &wfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = CONNECT_TIMEOUT_MS * 1000 };
        rc = select(s + 1, NULL, &wfds, NULL, &tv);
        if (rc == 1) {
            int err = 0;
            socklen_t elen = sizeof(err);
            getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &elen);
            rc = (err == 0) ? 0 : -1;
        } else {
            rc = -1;
        }
    }
    freeaddrinfo(res);

    if (rc != 0) {
        close(s);
        ESP_LOGW(TAG, "%s:%u connect failed", h->host, (unsigned)h->port);
        return false;
    }

    fcntl(s, F_SETFL, fl); /* back to blocking */
    struct timeval rcvto = { .tv_sec = 0, .tv_usec = REQUEST_TIMEOUT_MS * 1000 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto));
    int nodelay = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    h->sock = s;
    ESP_LOGI(TAG, "%s:%u connected", h->host, (unsigned)h->port);
    return true;
}

static bool recv_all(int s, uint8_t *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        int n = recv(s, buf + got, len - got, 0);
        if (n <= 0) {
            return false;
        }
        got += (size_t)n;
    }
    return true;
}

/* Read n_regs (1 or 2) registers — or, for func 1/2 (coil / discrete
 * input), ONE bit: regs[0] becomes 0/1. Returns false on any comms or
 * protocol error (caller closes the socket and backs the host off). */
static bool mb_read_regs(mb_host_t *h, const cfg_channel_t *c,
                         uint16_t n_regs, uint16_t *regs)
{
    bool bits = (c->func == 1 || c->func == 2);
    uint16_t quantity = bits ? 1 : n_regs; /* regs or bits, per func */

    uint16_t tid = ++s_tid;
    uint8_t req[12] = {
        (uint8_t)(tid >> 8), (uint8_t)tid, /* transaction id */
        0x00, 0x00,                        /* protocol id = 0 */
        0x00, 0x06,                        /* length: uid + pdu = 6 */
        c->unit,
        c->func,                           /* 1/2 = bits, 3/4 = registers */
        (uint8_t)(c->reg >> 8), (uint8_t)c->reg,
        (uint8_t)(quantity >> 8), (uint8_t)quantity,
    };
    if (send(h->sock, req, sizeof(req), 0) != (int)sizeof(req)) {
        return false;
    }

    uint8_t hdr[MBAP_LEN + 2]; /* MBAP + func + (bytecount | exception) */
    if (!recv_all(h->sock, hdr, sizeof(hdr))) {
        return false;
    }
    uint16_t rtid = ((uint16_t)hdr[0] << 8) | hdr[1];
    uint16_t rlen = ((uint16_t)hdr[4] << 8) | hdr[5];
    if (rtid != tid || rlen < 3) {
        ESP_LOGW(TAG, "%s: bad mbap (tid %u/%u len %u)", c->code,
                 (unsigned)rtid, (unsigned)tid, (unsigned)rlen);
        return false;
    }
    if (hdr[MBAP_LEN] & 0x80) { /* exception response: hdr[8] = code */
        ESP_LOGW(TAG, "%s: modbus exception %u", c->code, (unsigned)hdr[8]);
        /* drain nothing further: exception PDU is exactly 2 bytes, done */
        return false;
    }

    uint8_t bytecount = hdr[MBAP_LEN + 1];
    uint8_t expected = bits ? 1 : (uint8_t)(n_regs * 2);
    if (bytecount != expected || bytecount > 4) {
        ESP_LOGW(TAG, "%s: bad bytecount %u", c->code, (unsigned)bytecount);
        return false;
    }
    uint8_t data[4];
    if (!recv_all(h->sock, data, bytecount)) {
        return false;
    }
    if (bits) {
        regs[0] = data[0] & 0x01;
        regs[1] = 0;
        return true;
    }
    for (uint16_t i = 0; i < n_regs; i++) {
        regs[i] = ((uint16_t)data[2 * i] << 8) | data[2 * i + 1];
    }
    return true;
}

static float decode_value(const cfg_channel_t *c, const uint16_t *regs)
{
    /* two-register types default to big-endian word order (reg0 = high
     * word, ABCD); word_swap covers loword-first devices (CDAB). */
    uint32_t raw = c->word_swap ? (((uint32_t)regs[1] << 16) | regs[0])
                                : (((uint32_t)regs[0] << 16) | regs[1]);
    switch (c->dtype) {
    case DT_I16: return (float)(int16_t)regs[0];
    case DT_U32: return (float)raw;
    case DT_I32: return (float)(int32_t)raw;
    case DT_F32: {
        float f;
        memcpy(&f, &raw, sizeof(f));
        return f;
    }
    case DT_U16:
    default:     return (float)regs[0];
    }
}

/* func 1/2 read one bit; 32-bit register types need two registers */
static uint16_t regs_needed(const cfg_channel_t *c)
{
    if (c->func == 1 || c->func == 2) {
        return 1;
    }
    return (c->dtype == DT_U32 || c->dtype == DT_I32 ||
            c->dtype == DT_F32) ? 2 : 1;
}

static void poll_channel(const cfg_channel_t *c)
{
    if (c->host[0] == '\0') {
        return; /* unconfigured on the server; warned once at cfg load */
    }
    mb_host_t *h = host_for(c);
    if (h == NULL) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now < h->blocked_until_us) {
        return; /* host in backoff; poll again next due time */
    }
    if (h->sock < 0 && !host_connect(h)) {
        h->blocked_until_us = now + (int64_t)HOST_BACKOFF_MS * 1000;
        meas_push(c->id, SRC_MBTCP, Q_COMM_ERR, 0.0f);
        return;
    }

    uint16_t n_regs = regs_needed(c);
    uint16_t regs[2] = { 0 };
    if (!mb_read_regs(h, c, n_regs, regs)) {
        host_close(h);
        h->blocked_until_us = esp_timer_get_time() + (int64_t)HOST_BACKOFF_MS * 1000;
        /* value 0.0 not NAN: NAN is not representable in the JSON payload */
        meas_push(c->id, SRC_MBTCP, Q_COMM_ERR, 0.0f);
        return;
    }

    float v = decode_value(c, regs) * c->scale + c->offset;
    meas_push(c->id, SRC_MBTCP, Q_GOOD, v);
}

/* ---------------------------------------------------------------------
 * BLOCK READS — gom cac kenh cung (host, port, slave, func) co thanh ghi
 * gan nhau thanh MOT request FC03/04 (toi da 120 thanh ghi): 7 kenh
 * gateway = 1 vong TCP->RTU thay vi 7, giam han "cham do vat" do nghen.
 * Lo hong giua cac kenh (<= BLOCK_GAP_MAX) duoc doc gop roi BO — minh chi
 * trich dung offset cua kenh da khai. Thiet bi kho tinh tra exception cho
 * block -> tu DEMOTE (single=true), quay ve doc le nhu cu, khong te hon
 * hien trang. Func 1/2 (bit) luon doc le. Khong can khai gi tren Odoo. */
#define BLOCK_MAX_REGS 120  /* < gioi han Modbus 125 thanh ghi/request */
#define BLOCK_GAP_MAX  16   /* lo <=16 thanh ghi: van gom chung block */

typedef struct {
    uint8_t  members[CFG_MAX_CHANNELS]; /* chi so vao mang chans[] */
    uint8_t  n_members;
    uint16_t reg_start;
    uint16_t reg_count;
    uint32_t period_ms;                 /* min period cua cac kenh trong block */
    int64_t  next_due_us;
    bool     single;                    /* bit (fc1/2) hoac demote sau exception */
} mb_block_t;

static mb_block_t          s_blocks[CFG_MAX_CHANNELS];
static size_t              s_block_count;
static const cfg_channel_t *s_chans;
static size_t              s_chan_count;

static int chan_cmp(const cfg_channel_t *a, const cfg_channel_t *b)
{
    int r = strcmp(a->host, b->host);
    if (r != 0)             return r;
    if (a->port != b->port) return a->port < b->port ? -1 : 1;
    if (a->unit != b->unit) return a->unit < b->unit ? -1 : 1;
    if (a->func != b->func) return a->func < b->func ? -1 : 1;
    if (a->reg  != b->reg)  return a->reg  < b->reg  ? -1 : 1;
    return 0;
}

static void build_blocks(void)
{
    uint8_t order[CFG_MAX_CHANNELS];
    size_t  n = 0;
    for (size_t i = 0; i < s_chan_count && n < CFG_MAX_CHANNELS; i++) {
        if (s_chans[i].bus == BUS_MBTCP && s_chans[i].host[0] != '\0') {
            order[n++] = (uint8_t)i;
        }
    }
    for (size_t i = 1; i < n; i++) { /* insertion sort, n <= 32 */
        uint8_t k = order[i];
        size_t  j = i;
        while (j > 0 && chan_cmp(&s_chans[order[j - 1]], &s_chans[k]) > 0) {
            order[j] = order[j - 1];
            j--;
        }
        order[j] = k;
    }

    s_block_count = 0;
    mb_block_t *b  = NULL;
    uint32_t    end = 0; /* thanh ghi ket thuc (exclusive) cua block hien tai */
    for (size_t i = 0; i < n; i++) {
        const cfg_channel_t *c   = &s_chans[order[i]];
        uint16_t             need = regs_needed(c);
        bool                 bits = (c->func == 1 || c->func == 2);
        uint32_t             nend = c->reg + need > end ? c->reg + need : end;
        bool fits = false;
        if (b != NULL && !b->single && !bits) {
            const cfg_channel_t *c0 = &s_chans[b->members[0]];
            fits = strcmp(c0->host, c->host) == 0 && c0->port == c->port &&
                   c0->unit == c->unit && c0->func == c->func &&
                   (uint32_t)c->reg <= end + BLOCK_GAP_MAX &&
                   nend - b->reg_start <= BLOCK_MAX_REGS &&
                   b->n_members < CFG_MAX_CHANNELS;
        }
        if (!fits) {
            b = &s_blocks[s_block_count++];
            memset(b, 0, sizeof(*b));
            b->reg_start = c->reg;
            b->period_ms = c->period_ms ? c->period_ms : 2000;
            b->single    = bits;
            end          = c->reg;
            nend         = c->reg + need;
        }
        b->members[b->n_members++] = order[i];
        end          = nend;
        b->reg_count = (uint16_t)(end - b->reg_start);
        if (c->period_ms && c->period_ms < b->period_ms) {
            b->period_ms = c->period_ms;
        }
    }

    for (size_t i = 0; i < s_block_count; i++) {
        const mb_block_t    *bl = &s_blocks[i];
        const cfg_channel_t *c0 = &s_chans[bl->members[0]];
        if (bl->n_members > 1) {
            ESP_LOGI(TAG, "plan: %s:%u u%u fc%u regs %u..%u — %u kenh/1 request",
                     c0->host, (unsigned)c0->port, (unsigned)c0->unit,
                     (unsigned)c0->func, (unsigned)bl->reg_start,
                     (unsigned)(bl->reg_start + bl->reg_count - 1),
                     (unsigned)bl->n_members);
        } else {
            ESP_LOGI(TAG, "plan: %s:%u u%u fc%u reg %u (%s, doc le)",
                     c0->host, (unsigned)c0->port, (unsigned)c0->unit,
                     (unsigned)c0->func, (unsigned)c0->reg, c0->code);
        }
    }
}

typedef enum { RD_OK, RD_EXC, RD_COMM } rd_res_t;

/* FC03/04 doc `quantity` thanh ghi tu 1 request. RD_EXC = thiet bi tra
 * exception (transport van song — dung dong socket); RD_COMM = hong that. */
static rd_res_t mb_read_block(mb_host_t *h, const cfg_channel_t *c0,
                              uint16_t reg_start, uint16_t quantity,
                              uint16_t *out)
{
    uint16_t tid = ++s_tid;
    uint8_t req[12] = {
        (uint8_t)(tid >> 8), (uint8_t)tid,
        0x00, 0x00,
        0x00, 0x06,
        c0->unit,
        c0->func,
        (uint8_t)(reg_start >> 8), (uint8_t)reg_start,
        (uint8_t)(quantity >> 8), (uint8_t)quantity,
    };
    if (send(h->sock, req, sizeof(req), 0) != (int)sizeof(req)) {
        return RD_COMM;
    }
    uint8_t hdr[MBAP_LEN + 2];
    if (!recv_all(h->sock, hdr, sizeof(hdr))) {
        return RD_COMM;
    }
    uint16_t rtid = ((uint16_t)hdr[0] << 8) | hdr[1];
    uint16_t rlen = ((uint16_t)hdr[4] << 8) | hdr[5];
    if (rtid != tid || rlen < 3) {
        ESP_LOGW(TAG, "block %s u%u: bad mbap (tid %u/%u len %u)", c0->host,
                 (unsigned)c0->unit, (unsigned)rtid, (unsigned)tid,
                 (unsigned)rlen);
        return RD_COMM;
    }
    if (hdr[MBAP_LEN] & 0x80) {
        ESP_LOGW(TAG, "block %s u%u fc%u reg %u+%u: exception %u", c0->host,
                 (unsigned)c0->unit, (unsigned)c0->func, (unsigned)reg_start,
                 (unsigned)quantity, (unsigned)hdr[MBAP_LEN + 1]);
        return RD_EXC;
    }
    uint8_t bytecount = hdr[MBAP_LEN + 1];
    if (bytecount != (uint8_t)(quantity * 2)) {
        ESP_LOGW(TAG, "block %s u%u: bad bytecount %u (cho %u)", c0->host,
                 (unsigned)c0->unit, (unsigned)bytecount,
                 (unsigned)(quantity * 2));
        return RD_COMM;
    }
    uint8_t data[BLOCK_MAX_REGS * 2];
    if (!recv_all(h->sock, data, bytecount)) {
        return RD_COMM;
    }
    for (uint16_t i = 0; i < quantity; i++) {
        out[i] = ((uint16_t)data[2 * i] << 8) | data[2 * i + 1];
    }
    return RD_OK;
}

static void poll_block(mb_block_t *b)
{
    if (b->single) { /* bit channels / block bi demote: doc le nhu cu */
        for (uint8_t i = 0; i < b->n_members; i++) {
            poll_channel(&s_chans[b->members[i]]);
        }
        return;
    }

    const cfg_channel_t *c0 = &s_chans[b->members[0]];
    mb_host_t *h = host_for(c0);
    if (h == NULL) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now < h->blocked_until_us) {
        return;
    }
    if (h->sock < 0 && !host_connect(h)) {
        h->blocked_until_us = now + (int64_t)HOST_BACKOFF_MS * 1000;
        for (uint8_t i = 0; i < b->n_members; i++) {
            meas_push(s_chans[b->members[i]].id, SRC_MBTCP, Q_COMM_ERR, 0.0f);
        }
        return;
    }

    uint16_t buf[BLOCK_MAX_REGS];
    rd_res_t r = mb_read_block(h, c0, b->reg_start, b->reg_count, buf);
    if (r == RD_COMM) {
        host_close(h);
        h->blocked_until_us = esp_timer_get_time() +
                              (int64_t)HOST_BACKOFF_MS * 1000;
        for (uint8_t i = 0; i < b->n_members; i++) {
            meas_push(s_chans[b->members[i]].id, SRC_MBTCP, Q_COMM_ERR, 0.0f);
        }
        return;
    }
    if (r == RD_EXC) {
        /* thiet bi tu choi block (lo hong = o nho khong ton tai?):
         * demote vinh vien (den reboot) — an toan tuyet doi, cham nhat
         * bang hien trang doc le */
        b->single = true;
        ESP_LOGW(TAG, "block %s u%u regs %u..%u bi tu choi -> doc le %u kenh",
                 c0->host, (unsigned)c0->unit, (unsigned)b->reg_start,
                 (unsigned)(b->reg_start + b->reg_count - 1),
                 (unsigned)b->n_members);
        for (uint8_t i = 0; i < b->n_members; i++) {
            poll_channel(&s_chans[b->members[i]]);
        }
        return;
    }

    for (uint8_t i = 0; i < b->n_members; i++) {
        const cfg_channel_t *c = &s_chans[b->members[i]];
        float v = decode_value(c, &buf[c->reg - b->reg_start]) *
                  c->scale + c->offset;
        meas_push(c->id, SRC_MBTCP, Q_GOOD, v);
    }
}

static void poller_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    s_chans = cfg_get_channels(&s_chan_count);
    build_blocks();

    while (1) {
        int64_t now = esp_timer_get_time();
        int64_t soonest = now + 1000000;

        for (size_t i = 0; i < s_block_count; i++) {
            mb_block_t *b = &s_blocks[i];
            if (now >= b->next_due_us) {
                poll_block(b);
                b->next_due_us = esp_timer_get_time() +
                                 (int64_t)b->period_ms * 1000;
            }
            if (b->next_due_us < soonest) {
                soonest = b->next_due_us;
            }
        }

        esp_task_wdt_reset();
        int64_t sleep_ms = (soonest - esp_timer_get_time()) / 1000;
        if (sleep_ms < 10)   sleep_ms = 10;
        if (sleep_ms > 1000) sleep_ms = 1000; /* keep petting the wdt */
        vTaskDelay(pdMS_TO_TICKS(sleep_ms));
    }
}

esp_err_t mb_tcp_start(void)
{
    size_t count = 0;
    const cfg_channel_t *chans = cfg_get_channels(&count);
    bool any = false;
    for (size_t i = 0; i < count; i++) {
        if (chans[i].bus == BUS_MBTCP) {
            any = true;
            break;
        }
    }
    if (!any) {
        ESP_LOGI(TAG, "no mbtcp channels configured, poller not started");
        return ESP_OK;
    }

    /* 6KB stack: buf + data cua block read (2 x 240B) nam tren stack */
    BaseType_t ok = xTaskCreatePinnedToCore(poller_task, "mb_poll", 4096,
                                            NULL, 10, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
