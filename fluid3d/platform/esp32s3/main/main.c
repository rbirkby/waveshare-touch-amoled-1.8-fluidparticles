// Scheduling for the ESP32-S3. All the actual behaviour is in src/app.c.
//
// Task layout, one job per core so the two never fight:
//
//   core 1  sim      30 Hz Position-Based-Fluids solver
//   core 0  render   projects the latest snapshot and streams it to the panel
//   core 0  input    200 Hz IMU read and button poll (tiny, mostly idle)
//   core 0  stats    the 3 s timing line
//
// The sim and the renderer are deliberately decoupled: they exchange a single
// snapshot buffer under a mutex, so a slow frame never stalls the physics and a
// slow physics step never tears the picture.

#include "app.h"
#include "config.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void input_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    while (true) {
        app_input_poll();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    }
}

static void sim_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SIM_HZ);

    while (true) {
        app_sim_step();

        // Fixed timestep. If the solver ever overruns, the simulation simply
        // runs in slow motion instead of becoming unstable. vTaskDelayUntil
        // returns immediately in that case, which would starve the idle task
        // and trip the watchdog, so resynchronise and yield explicitly.
        const TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last) >= period) {
            last = now;
            vTaskDelay(1);
        } else {
            vTaskDelayUntil(&last, period);
        }
    }
}

static void render_task(void *arg)
{
    (void)arg;
    while (true) {
        app_render_frame();
        // Yield so the idle task can feed the watchdog.
        vTaskDelay(1);
    }
}

static void stats_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(APP_STATS_PERIOD_S * 1000));
        app_stats_tick();
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(app_init() == HAL_OK ? ESP_OK : ESP_FAIL);

    xTaskCreatePinnedToCore(sim_task, "sim", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(input_task, "input", 3072, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(stats_task, "stats", 3072, NULL, 1, NULL, 0);
}
