// Portable application logic. See app.h for why scheduling is not in here.

#include "app.h"

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "drivers.h"
#include "fluid.h"
#include "hal.h"
#include "render.h"

static const char *TAG = "fluid3d";

static bool s_have_imu;
static bool s_have_touch;
static volatile bool s_reset_requested;
static volatile uint32_t s_sim_us, s_render_us;
static volatile uint32_t s_sim_frames, s_render_frames;

hal_err_t app_init(void)
{
    hal_log(TAG, "%s", BOARD_NAME);
    hal_log(TAG, "3D fluid box: %dx%dx%d px, %d particles",
            LCD_H_RES, LCD_V_RES, BOX_D_PX, PARTICLE_COUNT);

    // Powers the rails, releases the panel and touch resets and opens I2C.
    // Everything order-dependent about a given board hides behind this call.
    if (board_init() != HAL_OK) {
        hal_log(TAG, "board init failed");
        return HAL_FAIL;
    }
    if (display_init() != HAL_OK) {
        hal_log(TAG, "display init failed");
        return HAL_FAIL;
    }

    s_have_touch = (touch_init() == HAL_OK);
    s_have_imu = (sensors_init() == HAL_OK);

    if (fluid_init() != HAL_OK || render_init() != HAL_OK) {
        return HAL_FAIL;
    }

    if (!s_have_imu) {
        hal_log(TAG, "no IMU, falling back to fixed downward gravity");
        fluid_set_motion(0.0f, -9.80665f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    return HAL_OK;
}

void app_input_poll(void)
{
    static int tick;
    sensors_sample_t s;

    if (s_have_imu && sensors_read(&s) == HAL_OK) {
        fluid_set_motion(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
    }
    if (button_pressed()) {
        hal_log(TAG, "user button: resetting simulation");
        s_reset_requested = true;
    }
    // The panel only refreshes every ~19 ms, so polling the touch controller
    // faster than that would just load the I2C bus for nothing.
    if (s_have_touch && (++tick & 3) == 0) {
        int tx, ty;
        const bool down = touch_read(&tx, &ty);
        fluid_set_touch(down, (float)tx, (float)ty);
    }
}

void app_sim_step(void)
{
    if (s_reset_requested) {
        s_reset_requested = false;
        fluid_reset();
    }
    const uint64_t t0 = hal_time_us();
    fluid_step();
    s_sim_us += (uint32_t)(hal_time_us() - t0);
    s_sim_frames++;
}

void app_render_frame(void)
{
    const uint64_t t0 = hal_time_us();
    render_frame();
    s_render_us += (uint32_t)(hal_time_us() - t0);
    s_render_frames++;
}

void app_stats_tick(void)
{
    const uint32_t sf = s_sim_frames, rf = s_render_frames;
    const uint32_t su = s_sim_us, ru = s_render_us;
    s_sim_frames = s_render_frames = s_sim_us = s_render_us = 0;

    const float period = (float)APP_STATS_PERIOD_S;
    hal_log(TAG, "sim %.1f Hz (%.1f ms)  render %.1f fps (%.1f ms)  free %u B",
            sf / period, sf ? su / (float)sf / 1000.0f : 0.0f,
            rf / period, rf ? ru / (float)rf / 1000.0f : 0.0f,
            (unsigned)hal_free_heap());
    if (sf) {
        hal_log(TAG, "  phases ms: ext %.1f grid %.1f neigh %.1f solve %.1f final %.1f",
                fluid_phase_us[0] / (float)sf / 1000.0f,
                fluid_phase_us[1] / (float)sf / 1000.0f,
                fluid_phase_us[2] / (float)sf / 1000.0f,
                fluid_phase_us[3] / (float)sf / 1000.0f,
                fluid_phase_us[4] / (float)sf / 1000.0f);
    }
    for (int i = 0; i < 5; i++) {
        fluid_phase_us[i] = 0;
    }
}
