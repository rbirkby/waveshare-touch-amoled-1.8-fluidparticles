// Portable application logic.
//
// This is everything the two boards do identically: bring the drivers up in the
// right order, feed motion and touch into the solver, step it at a fixed rate,
// draw it, and report timings. What is deliberately *not* here is scheduling.
// The ESP32-S3 has FreeRTOS and pins tasks to cores; the RP2350 runs a bare
// superloop with the renderer handed to core 1. Those are genuinely different
// mechanisms, so each platform's main.c owns its own, and calls the four hooks
// below at the right rates.
#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Brings up board, display, touch, IMU, solver and renderer, in that order.
hal_err_t app_init(void);

// Poll the IMU, the user button and the touch panel. Target ~200 Hz.
void app_input_poll(void);

// Advance the physics one fixed timestep. Call at exactly SIM_HZ.
void app_sim_step(void);

// Project and stream one frame to the panel. Call as fast as it will go.
void app_render_frame(void);

// Emit the timing line. Call every APP_STATS_PERIOD_S seconds.
#define APP_STATS_PERIOD_S 3
void app_stats_tick(void);

#ifdef __cplusplus
}
#endif
