// HAL implementation for the RP2350.
//
// The Pico SDK has no RTOS, so the pieces here are thinner than the ESP32-S3's:
// time comes from the always-on timer, allocation from the single newlib heap,
// and the mutex from pico_sync (which is inter-core safe, which matters because
// the solver and the renderer really do run on different cores).

#include "hal.h"

#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

#include "hardware/sync.h"
#include "pico/mutex.h"
#include "pico/rand.h"
#include "pico/stdlib.h"

extern char __StackLimit, __bss_end__; // provided by the SDK linker script

uint64_t hal_time_us(void)
{
    return to_us_since_boot(get_absolute_time());
}

void hal_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

void hal_log(const char *tag, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    printf("I (%llu) %s: ", (unsigned long long)(hal_time_us() / 1000), tag);
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

uint32_t hal_random_u32(void)
{
    return get_rand_32();
}

void *hal_alloc_fast(size_t bytes)
{
    return calloc(1, bytes);
}

void *hal_alloc_bulk(size_t bytes)
{
    // This board has no external PSRAM, so bulk and fast are the same 520 KB of
    // on-chip SRAM. That is ample: the whole application needs about 170 KB.
    return calloc(1, bytes);
}

void *hal_alloc_dma(size_t bytes)
{
    // Any SRAM is DMA-capable on the RP2350.
    return calloc(1, bytes);
}

size_t hal_free_heap(void)
{
    // The SDK grows the heap up towards the stack, so what is left is the gap
    // between the current break and the stack limit.
    struct mallinfo mi = mallinfo();
    const size_t total = (size_t)(&__StackLimit - &__bss_end__);
    return total - mi.uordblks;
}

hal_mutex_t *hal_mutex_create(void)
{
    mutex_t *m = calloc(1, sizeof(mutex_t));
    if (m != NULL) {
        mutex_init(m);
    }
    return (hal_mutex_t *)m;
}

void hal_mutex_lock(hal_mutex_t *m)
{
    mutex_enter_blocking((mutex_t *)m);
}

void hal_mutex_unlock(hal_mutex_t *m)
{
    mutex_exit((mutex_t *)m);
}
