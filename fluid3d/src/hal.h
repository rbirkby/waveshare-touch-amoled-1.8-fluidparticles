// Platform abstraction layer.
//
// Everything in src/ is portable C and may only talk to the hardware through
// this header. Each board under platform/<target>/ provides an implementation.
// The contract is deliberately tiny: the simulation needs a clock, a log, a
// random source, two flavours of allocation and one mutex, and that is all.
#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Return codes. Zero is success on every platform, which lets the drivers keep
// returning their native error type where that is convenient.
typedef int hal_err_t;
#define HAL_OK 0
#define HAL_FAIL (-1)

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
// Monotonic microseconds since boot. Must not wrap within a session.
uint64_t hal_time_us(void);

// Blocks the calling thread. Used only during initialisation.
void hal_delay_ms(uint32_t ms);

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
// printf-style, one line per call; the implementation appends the newline.
void hal_log(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// ---------------------------------------------------------------------------
// Random
// ---------------------------------------------------------------------------
// Only used to jitter the initial particle lattice, so it need not be strong.
uint32_t hal_random_u32(void);

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------
// Fast, always-resident RAM. Everything the solver touches per step lives here.
// Returns zeroed memory, or NULL. Never freed: all allocation happens at init.
void *hal_alloc_fast(size_t bytes);

// Bulk RAM for things touched once per frame. Maps to PSRAM where the board has
// it and falls back to hal_alloc_fast where it does not.
void *hal_alloc_bulk(size_t bytes);

// Memory that a display DMA engine can read from.
void *hal_alloc_dma(size_t bytes);

// Free internal heap in bytes, for the stats line.
size_t hal_free_heap(void);

// ---------------------------------------------------------------------------
// Mutual exclusion
// ---------------------------------------------------------------------------
// Guards the particle snapshot shared between the simulation and the renderer,
// which on both boards run on different cores.
typedef struct hal_mutex hal_mutex_t;

hal_mutex_t *hal_mutex_create(void);
void hal_mutex_lock(hal_mutex_t *m);
void hal_mutex_unlock(hal_mutex_t *m);

#ifdef __cplusplus
}
#endif
