#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

#include "diag.h"

static const char *TAG = "diag";

/* In bang tac vu moi 6 lan bao cao = 60 giay. Dong heap thi moi 10 giay:
 * no la cai ta doc thuong xuyen, bang tac vu thi doi rat cham. */
#define TASKS_EVERY 6

/* ------------------------------------------------------ day cua so -- */

static uint32_t s_win_min = UINT32_MAX;

void diag_tick(void)
{
    uint32_t f = esp_get_free_heap_size();
    if (f < s_win_min) {
        s_win_min = f;
    }
}

/* ------------------------------------------------------- bang tac vu -- */

#if CONFIG_FREERTOS_USE_TRACE_FACILITY

/* Cap phat MOT lan, tinh. Neu xin bo nho ngay trong cai dung cu do bo nho
 * thi chinh dung cu tro thanh thu no dang do. */
#define MAX_TASKS 32
static TaskStatus_t s_st[MAX_TASKS];

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#ifndef configRUN_TIME_COUNTER_TYPE
#define configRUN_TIME_COUNTER_TYPE uint32_t
#endif
typedef configRUN_TIME_COUNTER_TYPE rt_t;

/* Bo dem chay tich luy tu luc boot, nen phai giu ban truoc de lay hieu.
 * Khop theo handle chu khong theo ten: ten co the trung. */
static TaskHandle_t s_prev_h[MAX_TASKS];
static rt_t         s_prev_rt[MAX_TASKS];
static UBaseType_t  s_prev_n;
static rt_t         s_prev_total;

static rt_t prev_of(TaskHandle_t h)
{
    for (UBaseType_t i = 0; i < s_prev_n; i++) {
        if (s_prev_h[i] == h) {
            return s_prev_rt[i];
        }
    }
    return 0; /* tac vu moi sinh — coi nhu bat dau tu 0 */
}
#endif /* GENERATE_RUN_TIME_STATS */

static void report_tasks(void)
{
    rt_t total = 0;
    UBaseType_t n = uxTaskGetSystemState(s_st, MAX_TASKS, &total);
    if (n == 0) {
        ESP_LOGW(TAG, "uxTaskGetSystemState: 0 tac vu (MAX_TASKS qua nho?)");
        return;
    }

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    rt_t d_total = total - s_prev_total;
#endif

    /* In theo cum de mot dong con doc duoc tren man hinh serial. */
    char line[240];
    size_t len = 0;
    int in_line = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        unsigned stack_free = (unsigned)s_st[i].usStackHighWaterMark;
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        rt_t d = s_st[i].ulRunTimeCounter - prev_of(s_st[i].xHandle);
        unsigned cpu = d_total ? (unsigned)((uint64_t)d * 1000u / d_total) : 0u;
        int w = snprintf(line + len, sizeof(line) - len, "%s=%u/%u.%u%% ",
                         s_st[i].pcTaskName, stack_free, cpu / 10, cpu % 10);
#else
        int w = snprintf(line + len, sizeof(line) - len, "%s=%u ",
                         s_st[i].pcTaskName, stack_free);
#endif
        if (w < 0 || (size_t)w >= sizeof(line) - len) {
            if (in_line == 0) {
                continue; /* mot muc dai hon ca dong: bo, dung lap vo han */
            }
            ESP_LOGI(TAG, "tacvu %s", line);
            len = 0;
            in_line = 0;
            i--; /* in lai muc nay o dong sau */
            continue;
        }
        len += (size_t)w;
        if (++in_line == 5) {
            ESP_LOGI(TAG, "tacvu %s", line);
            len = 0;
            in_line = 0;
        }
    }
    if (in_line) {
        ESP_LOGI(TAG, "tacvu %s", line);
    }

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    s_prev_n = n < MAX_TASKS ? n : MAX_TASKS;
    for (UBaseType_t i = 0; i < s_prev_n; i++) {
        s_prev_h[i]  = s_st[i].xHandle;
        s_prev_rt[i] = s_st[i].ulRunTimeCounter;
    }
    s_prev_total = total;
#endif
}

#else  /* !CONFIG_FREERTOS_USE_TRACE_FACILITY */

static void report_tasks(void)
{
    ESP_LOGW(TAG, "bang tac vu tat: can CONFIG_FREERTOS_USE_TRACE_FACILITY=y");
}

#endif

/* ----------------------------------------------------------- bao cao -- */

void diag_report(void)
{
    static unsigned round;

    multi_heap_info_t hi;
    heap_caps_get_info(&hi, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    uint32_t now = esp_get_free_heap_size();
    uint32_t wmin = s_win_min <= now ? s_win_min : now;
    s_win_min = UINT32_MAX; /* dat lai — day la diem khac min_heap */

    /* dip = cua so 10 giay vua roi da tut sau bao nhieu duoi muc dang co.
     * Day moi la con so noi len rui ro that: no lap lai moi ngay, con
     * min_heap thi ke lai mot khoanh khac da qua tu lau. */
    ESP_LOGI(TAG,
             "heap int_free=%u int_largest=%u int_min=%u | win_min=%" PRIu32
             " dip=%" PRIu32,
             (unsigned)hi.total_free_bytes,
             (unsigned)hi.largest_free_block,
             (unsigned)hi.minimum_free_bytes,
             wmin, now > wmin ? now - wmin : 0);

    if (++round % TASKS_EVERY == 0) {
        report_tasks();
    }
}
