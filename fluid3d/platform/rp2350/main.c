// Scheduling for the RP2350. All the actual behaviour is in src/app.c.
//
// There is no RTOS here, so the split is done by hand across the two Cortex-M33
// cores, mirroring what FreeRTOS does on the ESP32-S3:
//
//   core 1  the 30 Hz solver, plus the 200 Hz input poll in its spare time
//   core 0  the renderer, flat out, plus the 3 s stats line
//
// Putting input on core 1 matters: a frame takes longer than an input tick, so
// polling from the render loop would quietly drop the IMU rate to the frame
// rate. Core 1 is idle between solver steps and has the time going spare.
//
// I2C is only ever touched from core 1 and the display only from core 0, so the
// two never contend for a peripheral. The one genuinely shared thing is the
// particle snapshot, and that is already guarded by a mutex inside the solver.

#include <stdio.h>

#include "app.h"
#include "config.h"
#include "hal.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

static void core1_main(void)
{
    const uint64_t sim_period_us = 1000000ull / SIM_HZ;
    const uint64_t input_period_us = 5000;
    uint64_t next_sim = hal_time_us();
    uint64_t next_input = next_sim;

    while (true) {
        const uint64_t now = hal_time_us();

        if (now >= next_sim) {
            app_sim_step();
            // Fixed timestep. If a step overruns, resynchronise rather than
            // trying to catch up: the simulation runs briefly in slow motion
            // instead of spiralling into an ever-growing backlog.
            next_sim = (now >= next_sim + sim_period_us) ? hal_time_us() + sim_period_us
                                                         : next_sim + sim_period_us;
        }
        if (now >= next_input) {
            app_input_poll();
            next_input = now + input_period_us;
        }
    }
}

int main(void)
{
    stdio_init_all();
    // Give the host a moment to reattach to the USB serial port, otherwise the
    // banner and any init errors are lost.
    sleep_ms(2000);

    if (app_init() != HAL_OK) {
        while (true) {
            printf("init failed\n");
            sleep_ms(1000);
        }
    }

    multicore_launch_core1(core1_main);

    uint64_t next_stats = hal_time_us() + APP_STATS_PERIOD_S * 1000000ull;
    while (true) {
        app_render_frame();
        if (hal_time_us() >= next_stats) {
            app_stats_tick();
            next_stats += APP_STATS_PERIOD_S * 1000000ull;
        }
    }
}
