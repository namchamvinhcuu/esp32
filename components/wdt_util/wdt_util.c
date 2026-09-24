#include "wdt_util.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

void wdt_safe_sleep_ms(uint32_t ms)
{
    while (ms > 0) {
        uint32_t step = ms > 1000 ? 1000 : ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        esp_task_wdt_reset();
        ms -= step;
    }
}
