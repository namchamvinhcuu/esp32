#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "mqtt_client.h"

#include "mqtt_link.h"

/* Tat trong menuconfig thi ca file thanh mot bo stub.
 *
 * Phai bao het ca file chu khong chi rieng mqtt_link_start(): cac lua chon
 * Kconfig khac deu `depends on MQTT_LINK_ENABLE`, nen khi tat chung KHONG
 * ton tai — moi cho tham chieu CONFIG_MQTT_LINK_BROKER_URI hay
 * CONFIG_MQTT_LINK_BATCH_MAX se lam vo build. */
#if !CONFIG_MQTT_LINK_ENABLE

esp_err_t mqtt_link_start(void)     { return ESP_OK; }
uint32_t  mqtt_link_published(void) { return 0; }
uint32_t  mqtt_link_dropped(void)   { return 0; }
uint32_t  mqtt_link_suppressed(void){ return 0; }
bool      mqtt_link_connected(void) { return false; }
void      mqtt_link_kick(void)      { }

#else

#include "cJSON.h"

#include "cfg.h"
#include "gpio_out.h"
#include "meas_core.h"
#include "net_mgr.h"
#include "spooler.h"
#include "wdt_util.h"

/* -- HAI CHE DO, mot file ------------------------------------------------
 *
 * CHE DO TAP (HTTP con bat) — giai doan do song song. Nguon so do la mot
 * cai "tap" gan vao meas_core. KHONG doc spooler: spool_peek() chi tra ve
 * nhung ban ghi cu nhat va uplink moi la ben xoa chung di, hai ben doc
 * chung mot vong dem thi HTTP ack truoc la MQTT mat ban ghi.
 *
 * CHE DO SPOOL (HTTP tat) — MQTT la duong duy nhat. Ly do tranh spooler o
 * tren bien mat cung voi uplink, va cai gia cua che do tap tro nen khong
 * chap nhan duoc: tap khong co bo nho, mat broker mot phut la mat that su
 * mot phut du lieu. Spooler giu 2048 ban ghi.
 *
 * Nen o che do spool, mqtt_link tiep quan dung vai tro cu cua uplink:
 * peek -> publish -> ack khi co PUBACK. Ack theo PUBACK chu khong theo
 * enqueue: enqueue chi noi "da bo vao hang cua tac vu mang", con outbox
 * cua esp-mqtt nam trong RAM va mat khi khoi dong lai. Chi PUBACK moi la
 * loi hua cua broker rang no da nhan.
 */
#if CONFIG_UPLINK_HTTP_ENABLE
#  define SPOOL_MODE 0
#else
#  define SPOOL_MODE 1
#endif

/* 256 ban ghi (5,6 KB) la thua: sau khi co bao-khi-doi nhip chi con ~1/s.
 * 32 van du hap thu mot con bung khi ca day chuyen doi trang thai cung luc. */
#define TAP_QUEUE_LEN 32
#define TOPIC_MAX     160

/* So kenh toi da theo doi cho RBE. cfg cho phep nhieu hon; kenh vuot bang
 * nay khong bi loc (luon phat) — an toan theo huong "phat thua con hon bo
 * sot". */
#define RBE_MAX_CHANNEL 32

/* Truoc moc nay coi nhu dong ho chua dong bo SNTP — giong het nguong
 * uplink.c dung, de hai duong gan nhan thoi gian nhu nhau. */
#define EPOCH_SANE_MS 1600000000000LL

/* Lenh tu edge: {"id":123,"channel":"relay_blue","cmd":"write","value":1}
 * — khoang 70 byte. 192 la rong rai; goi dai hon bi bo va ghi nhat ky. */
#define CMD_JSON_MAX  192
#define CMD_QUEUE_LEN 4

/* Cho PUBACK bao lau roi coi nhu mat va gui lai lo do. Chi dung o che do
 * spool. Gui lai co the sinh ban trung, nhung khoa chong trung cua may chu
 * la (serial, boot_id, seq) nen ban trung bi loai — mat ban ghi thi khong
 * cuu duoc, con ban trung thi co. */
#define PUBACK_TIMEOUT_MS 30000

static const char *TAG = "mqttlink";

#if !SPOOL_MODE
static QueueHandle_t            s_q;         /* chi che do tap */
#endif
static QueueHandle_t            s_cmd_q;
static TaskHandle_t             s_task;
static esp_mqtt_client_handle_t s_cli;
static volatile bool            s_connected;
static uint32_t                 s_published;
static uint32_t                 s_dropped;
static char                     s_topic_meas[TOPIC_MAX];
static char                     s_topic_status[TOPIC_MAX];
static char                     s_topic_cmd[TOPIC_MAX];
static char                     s_topic_cmd_ack[TOPIC_MAX];

#if SPOOL_MODE
static volatile int  s_inflight = -1;  /* msg_id dang cho PUBACK, -1 = trong */
static volatile bool s_acked;          /* PUBACK cua s_inflight da toi */
static int64_t       s_inflight_us;
static uint16_t      s_ack_bid;
static uint32_t      s_ack_seq;
#endif

typedef struct { char json[CMD_JSON_MAX]; } cmd_msg_t;

/* -- bao-khi-doi (report by exception) -----------------------------------
 *
 * Do ngay tren duong day nay 18/09: count1, pedal1, count2 phat so 0 moi
 * 200 ms va chiem 75% luu luong. Phat lai mot gia tri khong doi khong noi
 * them dieu gi, nhung no chiem song, va chinh loai lang phi do da lam sap
 * instance Odoo sang cung ngay.
 *
 * Quy tac: phat khi (a) gia tri doi qua vung chet, (b) chat luong doi,
 * hoac (c) da im qua lau.
 *
 * (c) KHONG duoc bo. Kenh ben Odoo co `max_age_ms`; im lang qua nguong do
 * thi o gia tri chuyen xam va nut [Dat] bi chan — da gap that voi
 * scale_esp32 (max_age_ms 1500 ms trong khi node day moi 5 s). Nen gia tri
 * dung yen van phai duoc nhac lai dinh ky.
 */
static float    s_rbe_val[RBE_MAX_CHANNEL];
static uint8_t  s_rbe_q[RBE_MAX_CHANNEL];
static int64_t  s_rbe_us[RBE_MAX_CHANNEL];
static bool     s_rbe_seen[RBE_MAX_CHANNEL];
static uint32_t s_suppressed;

static bool rbe_should_send(const measurement_t *m)
{
    if (m->channel_id >= RBE_MAX_CHANNEL) {
        return true;                    /* ngoai bang: khong loc */
    }
    const uint16_t i = m->channel_id;
    const int64_t now = esp_timer_get_time();

    if (!s_rbe_seen[i]) {
        goto send;                      /* mau dau tien cua kenh */
    }
    if (m->quality != s_rbe_q[i]) {
        goto send;
    }
    if ((now - s_rbe_us[i]) >= (int64_t)CONFIG_MQTT_LINK_MAX_SILENCE_MS * 1000) {
        goto send;                      /* nhac lai dinh ky */
    }
    {
        const float d = m->value - s_rbe_val[i];
        const float band = (float)CONFIG_MQTT_LINK_DEADBAND_MILLI / 1000.0f;
        if ((d < 0 ? -d : d) > band) {
            goto send;
        }
    }
    return false;

send:
    s_rbe_val[i]  = m->value;
    s_rbe_q[i]    = m->quality;
    s_rbe_us[i]   = now;
    s_rbe_seen[i] = true;
    return true;
}

/* Quen het, lan sau phat lai tat ca.
 *
 * Goi khi mot lo KHONG di duoc. rbe_should_send() da ghi "da phat gia tri
 * nay" ngay luc quyet dinh, nen neu lo do roi thi nhung ban ghi con nam
 * trong spooler se bi chinh cai vet do dan xuong o lan thu lai — ban ghi
 * con nguyen ma khong ai gui. Xoa vet di thi lan sau chung duoc phat lai. */
static void rbe_reset(void)
{
    memset(s_rbe_seen, 0, sizeof(s_rbe_seen));
}

#if !SPOOL_MODE
/* Chay TREN TASK DO (mb_tcp, scale_serial, io_scan...), nen tuyet doi
 * khong duoc chan. Hang doi day thi bo va dem — o che do tap HTTP van gui
 * du du lieu, mat o day khong mat that. */
static void tap_cb(const measurement_t *m)
{
    if (!rbe_should_send(m)) {
        __atomic_add_fetch(&s_suppressed, 1, __ATOMIC_RELAXED);
        return;
    }
    if (s_q != NULL && xQueueSend(s_q, m, 0) != pdTRUE) {
        __atomic_add_fetch(&s_dropped, 1, __ATOMIC_RELAXED);
    }
}
#endif

/* -- gom mot lo thanh JSON, KHONG dung cJSON -----------------------------
 *
 * Ban dau toi bat chuoc build_measurements_body() cua uplink.c: dung cay
 * cJSON roi in ra chuoi. Tren con node nay do la sai lam.
 *
 * Mot cay cJSON cho 50 ban ghi la ~350 nut x ~64 byte = ~22 KB NHAT THOI,
 * cong chuoi in ra, cong bo dem cua cJSON_Print tu nhan doi khi day. Ma
 * uplink cung dung dung cach do cung luc — hai cay tren mot heap 48 KB.
 * Ket qua do duoc 19/09: min_heap tut con 448 byte, upload_batch khong cap
 * phat noi bo dem HTTP, `sent=0` va spool day cung 2048.
 *
 * Goi nay co hinh dang co dinh, khong can cay doi tuong. Viet thang vao
 * mot bo dem TINH: khong cap phat, khong phan manh, tran thi cat lo.
 *
 * Hinh dang giu nguyen 100% so voi POST /node/v1/measurements — do van la
 * ly do consumer khong can bo phan tich thu hai.
 */
static char s_body[CONFIG_MQTT_LINK_BODY_BYTES];

/* Tra ve so ban ghi da viet duoc (co the < n neu bo dem day), 0 neu khong
 * viet duoc gi. Do dai chuoi nam o *out_len. */
static size_t build_body(const measurement_t *batch, size_t n, size_t *out_len)
{
    const int64_t offset = net_mgr_time_offset_ms();
    int w = snprintf(s_body, sizeof(s_body), "{\"bid\":%u,\"seq\":%" PRIu32 ",\"items\":[",
                     (unsigned)batch[n - 1].boot_id, batch[n - 1].seq);
    if (w < 0 || (size_t)w >= sizeof(s_body)) {
        return 0;
    }
    size_t len = (size_t)w;
    size_t done = 0;

    for (size_t i = 0; i < n; i++) {
        const measurement_t *m  = &batch[i];
        const cfg_channel_t *ch = cfg_channel_by_id(m->channel_id);
        if (ch == NULL) {
            continue;
        }
        int64_t ts = m->ts_ms;
        if (ts < EPOCH_SANE_MS) {
            ts += offset;   /* ghi truoc lan dong bo SNTP dau tien */
        }
        int k = snprintf(s_body + len, sizeof(s_body) - len,
                         "%s{\"ch\":\"%s\",\"v\":%.4f,\"s\":null,\"q\":%u,"
                         "\"ts\":%lld,\"stable\":%s}",
                         done ? "," : "", ch->code, (double)m->value,
                         (unsigned)m->quality, (long long)ts,
                         m->quality != Q_UNSTABLE ? "true" : "false");
        if (k < 0 || (size_t)k >= sizeof(s_body) - len) {
            break;          /* day bo dem: gui nhung gi da co, phan con lai
                             * o lai cho lo sau */
        }
        len += (size_t)k;
        done++;
    }
    if (done == 0) {
        return 0;
    }
    int k = snprintf(s_body + len, sizeof(s_body) - len, "]}");
    if (k < 0 || (size_t)k >= sizeof(s_body) - len) {
        return 0;
    }
    *out_len = len + (size_t)k;
    return done;
}

/* -- duong LENH (chieu xuong) --------------------------------------------
 *
 * Doi xung voi duong so do: edge phat len <goc>/<serial>/cmd, node tra lai
 * ket qua o <goc>/<serial>/cmdack. Day la cho lay lai 2,2 giay — truoc kia
 * node phai DI HOI GET /node/v1/commands moi 2 giay, va do tre trung binh
 * cua mot lan bam den chinh la nua chu ky do.
 *
 * KHONG dung retain va KHONG dung phien ben bi cho chieu nay, co chu dich:
 * mot lenh bat den gui luc node dang mat dien khong duoc phep tu bat len
 * khi no song lai nua tieng sau. Lenh la thu cua hien tai. Broker vut di
 * lenh gui cho mot node vang mat, dung nhu ta muon.
 *
 * Chu de ack la "cmdack" chu khong phai "cmd/ack" — mot tang, de ben doc
 * tach duoc serial ra khoi chu de bang cung mot phep tach nhu meas/status.
 */
static void publish_cmd_ack(long id, bool ok, const char *detail)
{
    char body[128];
    int n;
    if (ok) {
        n = snprintf(body, sizeof(body), "{\"id\":%ld,\"ok\":true}", id);
    } else {
        n = snprintf(body, sizeof(body), "{\"id\":%ld,\"ok\":false,\"detail\":\"%s\"}",
                     id, detail ? detail : "");
    }
    if (n > 0 && (size_t)n < sizeof(body)) {
        esp_mqtt_client_enqueue(s_cli, s_topic_cmd_ack, body, n, 1, 0, true);
    }
}

/* Thuc thi tren link_task chu KHONG tren tac vu su kien cua esp-mqtt:
 * ngan xep cua no chi 3584 byte va con phai chay ca dong giao thuc. Phan
 * tich cJSON cong ghi GPIO o do la cach chac chan de tran ngan xep. */
static void handle_command(const char *json)
{
    static long s_last_id = -1;

    cJSON *r = cJSON_Parse(json);
    if (r == NULL) {
        ESP_LOGW(TAG, "lenh khong phai JSON hop le");
        return;
    }
    const cJSON *jid = cJSON_GetObjectItemCaseSensitive(r, "id");
    if (!cJSON_IsNumber(jid)) {
        cJSON_Delete(r);
        return;
    }
    const long id = (long)jid->valuedouble;

    /* Gui lai cung mot id (QoS 1 cho phep trung) thi ACK lai nhung KHONG
     * thuc thi lai — bam mot lan khong duoc bien thanh hai lan dao relay. */
    if (id == s_last_id) {
        ESP_LOGI(TAG, "lenh %ld da thuc thi roi, chi ack lai", id);
        publish_cmd_ack(id, true, NULL);
        cJSON_Delete(r);
        return;
    }

    const cJSON *ch    = cJSON_GetObjectItemCaseSensitive(r, "channel");
    const cJSON *op    = cJSON_GetObjectItemCaseSensitive(r, "cmd");
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(r, "value");
    const cJSON *jms   = cJSON_GetObjectItemCaseSensitive(r, "ms");
    const cJSON *jper  = cJSON_GetObjectItemCaseSensitive(r, "period_ms");
    const char *ch_code = cJSON_IsString(ch) ? ch->valuestring : "";
    const char *op_str  = cJSON_IsString(op) ? op->valuestring : "";

    const gpio_cmd_t gc = {
        .channel   = ch_code,
        .op        = op_str,
        .has_value = cJSON_IsNumber(value),
        .value     = cJSON_IsNumber(value) ? value->valuedouble : 0.0,
        .ms        = cJSON_IsNumber(jms)  ? (int32_t)jms->valuedouble  : 0,
        .period_ms = cJSON_IsNumber(jper) ? (int32_t)jper->valuedouble : 0,
    };
    char detail[64];
    bool ok = gpio_out_execute(&gc, detail, sizeof(detail));
    if (ok) {
        s_last_id = id;
        ESP_LOGI(TAG, "lenh %ld [%s] tren kenh %s: OK", id, op_str, ch_code);
    } else {
        ESP_LOGW(TAG, "lenh %ld [%s] tren kenh %s bi tu choi: %s",
                 id, op_str, ch_code, detail);
    }
    publish_cmd_ack(id, ok, detail);
    cJSON_Delete(r);
}

static void drain_commands(void)
{
    cmd_msg_t msg;
    while (s_cmd_q != NULL && xQueueReceive(s_cmd_q, &msg, 0) == pdTRUE) {
        handle_command(msg.json);
        esp_task_wdt_reset();
    }
}

static void on_mqtt_event(void *handler_args, esp_event_base_t base,
                          int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    const esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        /* Dang ky o DAY chu khong o luc khoi tao: moi lan noi lai broker la
         * mot phien moi, dang ky cu khong con. Dat nham cho nay thi lenh
         * chay binh thuong toi lan rot mang dau tien roi im vinh vien. */
        esp_mqtt_client_subscribe(s_cli, s_topic_cmd, 1);
        /* Bao con song, retained: ai dang ky sau van doc duoc trang thai
         * hien tai ma khong phai cho goi ke tiep. Cap voi Last Will o
         * duoi — broker tu phat "online:false" khi node rot, khong ton
         * mot goi heartbeat nao. Day la thu HTTP khong lam duoc. */
        /* "cmd":true la loi TU GIOI THIEU: firmware nay biet nhan lenh qua
         * MQTT. Edge doc co nay de quyet dinh gui lenh xuong duong nao —
         * node cu (chi biet GET /node/v1/commands) khong co co nay nen van
         * duoc phuc vu bang hang doi poll. Khong phai dat cau hinh o hai
         * noi roi cho chung lech nhau. */
        esp_mqtt_client_publish(s_cli, s_topic_status,
                                "{\"online\":true,\"cmd\":true}", 0, 1, 1);
        ESP_LOGI(TAG, "da noi broker, dang ky %s", s_topic_cmd);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "mat ket noi broker");
        break;

    case MQTT_EVENT_DATA:
        /* Goi bi cat lam nhieu manh thi bo — lenh chi vai chuc byte, mot
         * goi lenh dai hon buffer.size (512) la goi hong hoac khong phai
         * cua ta. Ghep manh o day se phai giu trang thai giua cac lan goi
         * va khong dang cho mot thu khong bao gio xay ra. */
        if (e->data_len != e->total_data_len || e->data_len <= 0 ||
            (size_t)e->data_len >= CMD_JSON_MAX) {
            break;
        }
        if ((size_t)e->topic_len != strlen(s_topic_cmd) ||
            strncmp(e->topic, s_topic_cmd, (size_t)e->topic_len) != 0) {
            break;
        }
        {
            cmd_msg_t msg;
            memcpy(msg.json, e->data, (size_t)e->data_len);
            msg.json[e->data_len] = '\0';
            if (s_cmd_q != NULL) {
                xQueueSend(s_cmd_q, &msg, 0);
                if (s_task != NULL) {
                    xTaskNotifyGive(s_task);   /* thuc day de thuc thi ngay */
                }
            }
        }
        break;

#if SPOOL_MODE
    case MQTT_EVENT_PUBLISHED:
        /* PUBACK: broker da nhan. Gio moi duoc xoa khoi spooler. */
        if (s_inflight >= 0 && e->msg_id == s_inflight) {
            s_acked = true;
            if (s_task != NULL) {
                xTaskNotifyGive(s_task);
            }
        }
        break;
#endif

    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "loi mqtt");
        break;

    default:
        break;
    }
}

#if !SPOOL_MODE
/* -- che do tap: gom tu hang doi cua tap roi phat ------------------------ */
static void pump_tap(measurement_t *batch)
{
    /* Gom toi khi day lo HOAC het thoi gian cho — cai nao truoc.
     * FLUSH_MS nho thi so ve nhanh, nhung goi vun; day la cho chinh
     * de danh doi giua do tuoi va so luong goi. */
    size_t     n        = 0;
    TickType_t deadline = xTaskGetTickCount() +
                          pdMS_TO_TICKS(CONFIG_MQTT_LINK_FLUSH_MS);
    while (n < (size_t)CONFIG_MQTT_LINK_BATCH_MAX) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            break;
        }
        TickType_t wait = deadline - now;
        if (wait > pdMS_TO_TICKS(200)) {
            wait = pdMS_TO_TICKS(200); /* lat nho de con reset wdt */
        }
        if (xQueueReceive(s_q, &batch[n], wait) == pdTRUE) {
            n++;
        }
        esp_task_wdt_reset();
    }
    esp_task_wdt_reset();

    if (n == 0) {
        return;
    }
    if (!s_connected) {
        __atomic_add_fetch(&s_dropped, n, __ATOMIC_RELAXED);
        return;
    }

    size_t body_len = 0;
    size_t done = build_body(batch, n, &body_len);
    if (done == 0) {
        __atomic_add_fetch(&s_dropped, n, __ATOMIC_RELAXED);
        return;
    }
    /* enqueue, KHONG publish.
     *
     * esp_mqtt_client_publish() o QoS 1 CHAN tac vu goi cho toi khi co
     * PUBACK. enqueue() bo goi vao hang cua tac vu mang roi tra ve ngay;
     * QoS 1 va PUBACK van nguyen. esp-mqtt CHEP payload vao outbox cua no,
     * nen dung bo dem tinh o day la an toan. */
    int msg_id = esp_mqtt_client_enqueue(s_cli, s_topic_meas, s_body,
                                         (int)body_len, 1, 0, true);
    if (msg_id < 0) {
        __atomic_add_fetch(&s_dropped, done, __ATOMIC_RELAXED);
        ESP_LOGW(TAG, "enqueue that bai, bo %u ban ghi", (unsigned)done);
    } else {
        /* Dem `done`, KHONG phai `n`: build_body co the cat lo. Truoc
         * 19/09 o day cong CA HAI, nen con so nay bao gap doi su that. */
        __atomic_add_fetch(&s_published, done, __ATOMIC_RELAXED);
    }
    if (done < n) {
        __atomic_add_fetch(&s_dropped, n - done, __ATOMIC_RELAXED);
    }
}
#endif /* !SPOOL_MODE */

#if SPOOL_MODE
/* -- che do spool: peek -> phat -> ack khi co PUBACK --------------------- */
static void pump_spool(measurement_t *batch)
{
    /* Mot lo bay mot luc. Don gian, va tu no lam luon viec dieu nhip: khi
     * duong day ban thi lo sau to hon, khi vang thi goi di ngay. */
    if (s_inflight >= 0) {
        if (s_acked) {
            spool_ack_through(s_ack_bid, s_ack_seq);
            s_inflight = -1;
            s_acked = false;
        } else if (esp_timer_get_time() - s_inflight_us >
                   (int64_t)PUBACK_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "khong thay PUBACK sau %d ms, gui lai lo",
                     PUBACK_TIMEOUT_MS);
            s_inflight = -1;
            rbe_reset();
        } else {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
            return;
        }
    }
    if (!s_connected) {
        /* Khong dong vao spooler: ban ghi nam nguyen do cho ket noi lai.
         * Day chinh la thu che do tap khong co. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        return;
    }

    size_t n = spool_peek(batch, CONFIG_MQTT_LINK_BATCH_MAX);
    if (n == 0) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONFIG_MQTT_LINK_FLUSH_MS));
        return;
    }

    /* Bao-khi-doi tren duong spool: loc TAI CHO, giu lai moc cuoi cua ca
     * lo de con ack — ban ghi bi nen lai van phai bien khoi spooler, neu
     * khong lo sau se doc lai dung chung va vong nay khong bao gio tien. */
    const uint16_t tail_bid = batch[n - 1].boot_id;
    const uint32_t tail_seq = batch[n - 1].seq;
    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
        if (rbe_should_send(&batch[i])) {
            batch[k++] = batch[i];
        } else {
            __atomic_add_fetch(&s_suppressed, 1, __ATOMIC_RELAXED);
        }
    }
    if (k == 0) {
        spool_ack_through(tail_bid, tail_seq);
        return;
    }

    size_t body_len = 0;
    size_t done = build_body(batch, k, &body_len);
    if (done == 0) {
        /* Mot ban ghi don khong lot noi bo dem — khong the xay ra voi
         * BODY_BYTES tu 512 tro len, nhung neu xay ra thi phai nhich len,
         * khong duoc ket vinh vien o cung mot ban ghi. */
        ESP_LOGE(TAG, "khong dung noi goi, bo 1 ban ghi (kenh %u)",
                 (unsigned)batch[0].channel_id);
        __atomic_add_fetch(&s_dropped, 1, __ATOMIC_RELAXED);
        spool_ack_through(batch[0].boot_id, batch[0].seq);
        return;
    }

    int msg_id = esp_mqtt_client_enqueue(s_cli, s_topic_meas, s_body,
                                         (int)body_len, 1, 0, true);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "enqueue that bai, giu lo lai trong spooler");
        rbe_reset();
        vTaskDelay(pdMS_TO_TICKS(200));
        return;                       /* KHONG ack: lo do van con nguyen */
    }
    /* Ack toi dau: neu build_body cat lo thi chi toi ban ghi cuoi da ma
     * hoa — nhung ban ghi truoc no hoac da ma hoa hoac da bi nen lai, nen
     * xoa toi do la dung. */
    s_ack_bid = (done == k) ? tail_bid : batch[done - 1].boot_id;
    s_ack_seq = (done == k) ? tail_seq : batch[done - 1].seq;
    s_inflight_us = esp_timer_get_time();
    s_acked = false;
    s_inflight = msg_id;
    __atomic_add_fetch(&s_published, done, __ATOMIC_RELAXED);
}
#endif /* SPOOL_MODE */

static void link_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    s_task = xTaskGetCurrentTaskHandle();

    /* Cho co IP roi hay mo ket noi. esp-mqtt tu thu lai duoc, nhung cho
     * o day thi nhat ky sach hon va khong ban mot vong thu vo ich luc
     * moi boot. Cho theo lat nho de con nuoi watchdog. */
    while ((net_mgr_bits() & NET_BIT_WIFI) == 0) {
        wdt_safe_sleep_ms(500);
    }

    esp_mqtt_client_config_t cc = {
        .broker.address.uri            = CONFIG_MQTT_LINK_BROKER_URI,
        .credentials.client_id         = cfg_node_serial(),
        .credentials.username          = CONFIG_MQTT_LINK_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_LINK_PASSWORD,
        .session.keepalive             = 30,
        .session.last_will.topic       = s_topic_status,
        .session.last_will.msg         = "{\"online\":false}",
        .session.last_will.qos         = 1,
        .session.last_will.retain      = 1,
        .network.reconnect_timeout_ms  = 5000,
        /* Mac dinh cua esp-mqtt la 1024/1024 va ngan xep 6144 — rong rai
         * cho mot thiet bi thong thuong, qua tay cho con node nay. Goi ra
         * lon nhat la s_body; goi vao chi la PUBACK va lenh. */
        .buffer.size                   = 512,
        .buffer.out_size               = CONFIG_MQTT_LINK_BODY_BYTES + 256,
        .task.stack_size               = 3584,
    };

    s_cli = esp_mqtt_client_init(&cc);
    if (s_cli == NULL) {
        ESP_LOGE(TAG, "khong tao duoc client, dung component");
        esp_task_wdt_delete(NULL);
        vTaskDelete(NULL);
        return;
    }
    esp_mqtt_client_register_event(s_cli, ESP_EVENT_ANY_ID, on_mqtt_event, NULL);
    esp_mqtt_client_start(s_cli);

    measurement_t *batch = calloc(CONFIG_MQTT_LINK_BATCH_MAX,
                                  sizeof(measurement_t));
    if (batch == NULL) {
        ESP_LOGE(TAG, "khong du RAM cho lo %d ban ghi",
                 CONFIG_MQTT_LINK_BATCH_MAX);
        esp_task_wdt_delete(NULL);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        esp_task_wdt_reset();
        drain_commands();
#if SPOOL_MODE
        pump_spool(batch);
#else
        pump_tap(batch);
#endif
    }
}

esp_err_t mqtt_link_start(void)
{
    if (CONFIG_MQTT_LINK_BROKER_URI[0] == '\0') {
#if SPOOL_MODE
        ESP_LOGE(TAG, "chua dat broker URI ma HTTP cung tat — node se cam");
#else
        ESP_LOGW(TAG, "chua dat broker URI, bo qua (node van chay HTTP)");
#endif
        return ESP_OK;
    }

    const char *serial = cfg_node_serial();
    int tw;
    tw = snprintf(s_topic_meas, sizeof(s_topic_meas), "%s/%s/meas",
                  CONFIG_MQTT_LINK_TOPIC_BASE, serial);
    if (tw < 0 || (size_t)tw >= sizeof(s_topic_meas)) {
        ESP_LOGE(TAG, "topic base qua dai, khong the dung MQTT");
        return ESP_ERR_INVALID_SIZE;
    }
    tw = snprintf(s_topic_status, sizeof(s_topic_status), "%s/%s/status",
                  CONFIG_MQTT_LINK_TOPIC_BASE, serial);
    if (tw < 0 || (size_t)tw >= sizeof(s_topic_status)) {
        ESP_LOGE(TAG, "topic base qua dai, khong the dung MQTT");
        return ESP_ERR_INVALID_SIZE;
    }
    tw = snprintf(s_topic_cmd, sizeof(s_topic_cmd), "%s/%s/cmd",
                  CONFIG_MQTT_LINK_TOPIC_BASE, serial);
    if (tw < 0 || (size_t)tw >= sizeof(s_topic_cmd)) {
        ESP_LOGE(TAG, "topic base qua dai, khong the dung MQTT");
        return ESP_ERR_INVALID_SIZE;
    }
    tw = snprintf(s_topic_cmd_ack, sizeof(s_topic_cmd_ack), "%s/%s/cmdack",
                  CONFIG_MQTT_LINK_TOPIC_BASE, serial);
    if (tw < 0 || (size_t)tw >= sizeof(s_topic_cmd_ack)) {
        ESP_LOGE(TAG, "topic base qua dai, khong the dung MQTT");
        return ESP_ERR_INVALID_SIZE;
    }

    s_cmd_q = xQueueCreate(CMD_QUEUE_LEN, sizeof(cmd_msg_t));
    if (s_cmd_q == NULL) {
        return ESP_ERR_NO_MEM;
    }

#if !SPOOL_MODE
    s_q = xQueueCreate(TAP_QUEUE_LEN, sizeof(measurement_t));
    if (s_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = meas_add_tap(tap_cb);
    if (err != ESP_OK) {
        vQueueDelete(s_q);
        s_q = NULL;
        return err;
    }
    /* Uu tien 4 — THAP HON uplink (5) va cung loi 0: khi hai ben tranh
     * song thi duong chinh thuc phai thang. */
    const UBaseType_t prio = 4;
#else
    /* HTTP tat: khong con ai de nhuong. Lay dung uu tien cu cua uplink. */
    const UBaseType_t prio = 5;
#endif

    /* 4096: so do van viet bang snprintf vao bo dem TINH (khong cay cJSON),
     * nhung duong LENH co phan tich cJSON chay tren dung task nay. */
    BaseType_t ok = xTaskCreatePinnedToCore(link_task, "mqtt_link", 3072,
                                            NULL, prio, NULL, 0);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "che do %s; phat %s, nhan lenh %s",
             SPOOL_MODE ? "SPOOL (MQTT la duong duy nhat)"
                        : "TAP (chay song song HTTP)",
             s_topic_meas, s_topic_cmd);
    return ESP_OK;
}

uint32_t mqtt_link_published(void)
{
    return __atomic_load_n(&s_published, __ATOMIC_RELAXED);
}

uint32_t mqtt_link_dropped(void)
{
    return __atomic_load_n(&s_dropped, __ATOMIC_RELAXED);
}

uint32_t mqtt_link_suppressed(void)
{
    return __atomic_load_n(&s_suppressed, __ATOMIC_RELAXED);
}

bool mqtt_link_connected(void)
{
    return s_connected;
}

void mqtt_link_kick(void)
{
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
}

#endif /* CONFIG_MQTT_LINK_ENABLE */
