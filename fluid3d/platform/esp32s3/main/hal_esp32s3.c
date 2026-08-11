// HAL implementation for the ESP32-S3.
//
// Thin adapters onto ESP-IDF and FreeRTOS. The only interesting choices are
// which heap each allocator uses: the solver's working set must be in internal
// SRAM (PSRAM is far too slow for the random access the neighbour search does),
// while the once-per-frame snapshot is happy in PSRAM.

#include "hal.h"

#include <stdio.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

uint64_t hal_time_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

void hal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void hal_log(const char *tag, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    printf("I (%lu) %s: ", (unsigned long)(esp_timer_get_time() / 1000), tag);
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

uint32_t hal_random_u32(void)
{
    return esp_random();
}

void *hal_alloc_fast(size_t bytes)
{
    return heap_caps_calloc(1, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

void *hal_alloc_bulk(size_t bytes)
{
    // This board has 8 MB of octal PSRAM. Fall back to internal RAM if the
    // PSRAM is missing or exhausted.
    void *p = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM);
    return p ? p : hal_alloc_fast(bytes);
}

void *hal_alloc_dma(size_t bytes)
{
    return heap_caps_calloc(1, bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
}

size_t hal_free_heap(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
}

hal_mutex_t *hal_mutex_create(void)
{
    return (hal_mutex_t *)xSemaphoreCreateMutex();
}

void hal_mutex_lock(hal_mutex_t *m)
{
    xSemaphoreTake((SemaphoreHandle_t)m, portMAX_DELAY);
}

void hal_mutex_unlock(hal_mutex_t *m)
{
    xSemaphoreGive((SemaphoreHandle_t)m);
}
