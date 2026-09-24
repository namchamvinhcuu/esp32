#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "spooler.h"

#define CAP_PSRAM    4096 /* ~90 KB in PSRAM */
#define CAP_INTERNAL 2048 /* ~45 KB fallback on no-PSRAM devkits */

static const char *TAG = "spool";

static measurement_t   *s_ring;
static size_t           s_cap;
static size_t           s_head;  /* oldest */
static size_t           s_count;
static SemaphoreHandle_t s_mtx;
static uint32_t          s_overwritten;

static void ring_append(const measurement_t *m)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (s_count == s_cap) {
        /* full: overwrite oldest — bounded loss, oldest-first */
        s_head = (s_head + 1) % s_cap;
        s_count--;
        s_overwritten++;
    }
    s_ring[(s_head + s_count) % s_cap] = *m;
    s_count++;
    xSemaphoreGive(s_mtx);
}

static void drain_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    measurement_t m;
    while (1) {
        if (xQueueReceive(meas_queue(), &m, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ring_append(&m);
        }
        esp_task_wdt_reset();
    }
}

esp_err_t spooler_init(void)
{
    s_cap  = CAP_PSRAM;
    s_ring = heap_caps_malloc(s_cap * sizeof(measurement_t), MALLOC_CAP_SPIRAM);
    if (s_ring == NULL) {
        s_cap  = CAP_INTERNAL;
        s_ring = heap_caps_malloc(s_cap * sizeof(measurement_t), MALLOC_CAP_8BIT);
    }
    if (s_ring == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_mtx = xSemaphoreCreateMutex();
    if (s_mtx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* TODO(LittleFS): replace ring with flash segment spool, same API */
    ESP_LOGI(TAG, "ring ready: %u records (%u KB)", (unsigned)s_cap,
             (unsigned)(s_cap * sizeof(measurement_t) / 1024));

    BaseType_t ok = xTaskCreatePinnedToCore(drain_task, "spool_drain", 2048,
                                            NULL, 9, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

size_t spool_depth(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    size_t n = s_count;
    xSemaphoreGive(s_mtx);
    return n;
}

size_t spool_peek(measurement_t *out, size_t max_n)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    size_t n = (s_count < max_n) ? s_count : max_n;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(s_head + i) % s_cap];
    }
    xSemaphoreGive(s_mtx);
    return n;
}

void spool_ack_through(uint16_t boot_id, uint32_t seq)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    while (s_count > 0) {
        const measurement_t *oldest = &s_ring[s_head];
        if (oldest->boot_id != boot_id || oldest->seq > seq) {
            break;
        }
        s_head = (s_head + 1) % s_cap;
        s_count--;
    }
    xSemaphoreGive(s_mtx);
}

uint32_t spool_overwritten_count(void)
{
    return __atomic_load_n(&s_overwritten, __ATOMIC_RELAXED);
}
