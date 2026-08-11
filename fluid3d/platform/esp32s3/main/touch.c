#include "drivers.h"
#include "board.h"

#include <stdbool.h>

#include "board.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "touch";

static i2c_master_dev_handle_t s_dev;
static bool s_was_down;
static bool s_swap_axes;
static bool s_axes_known;

static void touch_clear_down(void)
{
    s_was_down = false;
}

// The touch controller's reset line is not a GPIO - it hangs off the TCA9554 IO
// expander, held asserted at power-on. Until it is released the controller does
// not acknowledge its own I2C address at all, which is why a plain bus scan of
// this board reports no touch hardware. The same expander also drives the panel
// reset, so this has to run before the display is brought up.
esp_err_t touch_release_reset(void)
{
    i2c_master_dev_handle_t io = board_i2c_add(ADDR_IO_EXPANDER);
    if (io == NULL) {
        ESP_LOGW(TAG, "IO expander missing, touch will stay in reset");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = board_reg_write(io, TCA9554_REG_CONFIG, (uint8_t)~TCA9554_OUTPUT_MASK);
    if (err == ESP_OK) {
        // Drive every reset low together, then release them together.
        err = board_reg_write(io, TCA9554_REG_OUTPUT, TCA9554_SD_CS);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (err == ESP_OK) {
        err = board_reg_write(io, TCA9554_REG_OUTPUT, TCA9554_OUTPUT_MASK);
    }
    vTaskDelay(pdMS_TO_TICKS(150));

    i2c_master_bus_rm_device(io);
    return err;
}

hal_err_t touch_init(void)
{
    s_dev = board_i2c_add(ADDR_TOUCH_CST816);
    if (s_dev == NULL) {
        s_dev = board_i2c_add(ADDR_TOUCH_FT3168);
        if (s_dev == NULL) {
            ESP_LOGW(TAG, "no touch controller found");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGI(TAG, "FT3168 at 0x%02X", ADDR_TOUCH_FT3168);
    } else {
        ESP_LOGI(TAG, "CST816 at 0x%02X", ADDR_TOUCH_CST816);
    }

    // The controller drops into a low-power state when nothing is touching it
    // and stops acknowledging its address, which produces a stream of I2C NACK
    // errors if it is polled blindly. Its interrupt line is the gate: pulled up
    // when idle, driven low while a finger is down. Watching the pin means the
    // bus is only used when there is something to read.
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << TOUCH_PIN_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    // An address ACK only proves something is on the bus. Reading the chip ID
    // back proves the controller is actually out of reset and talking.
    uint8_t id = 0;
    if (board_reg_read(s_dev, TOUCH_REG_CHIP_ID, &id, 1) == ESP_OK) {
        ESP_LOGI(TAG, "chip id 0x%02X", id);
    } else {
        ESP_LOGW(TAG, "chip id read failed");
    }
    return ESP_OK;
}

// Both the CST816 and the FT3168 use the same FocalTech-style register block:
// 0x02 is the contact count, then each contact is four bytes of big-endian
// 12-bit X and Y with flag bits in the top nibble.
bool touch_read(int *x, int *y)
{
    if (s_dev == NULL) {
        return false;
    }
    if (gpio_get_level(TOUCH_PIN_INT) != 0) {
        s_was_down = false;
        return false;
    }
    uint8_t b[5];
    if (board_reg_read(s_dev, TOUCH_REG_STATUS, b, sizeof(b)) != ESP_OK) {
        s_was_down = false;
        return false;
    }
    if ((b[0] & 0x0F) == 0) {
        touch_clear_down();
        return false;
    }

    int tx = (int)(((uint16_t)(b[1] & 0x0F) << 8) | b[2]);
    int ty = (int)(((uint16_t)(b[3] & 0x0F) << 8) | b[4]);

    // Logged once per touch-down so the panel-to-controller axis mapping can be
    // checked against where the finger actually was.
    // The panel is taller than it is wide, so a coordinate pair that only fits
    // when transposed is proof that this controller reports its axes the other
    // way round. Latch that once rather than silently dropping every touch
    // below y = 368, which is what a fixed guess would do if it guessed wrong.
    if (!s_axes_known && tx >= LCD_H_RES && tx < LCD_V_RES && ty < LCD_H_RES) {
        s_swap_axes = true;
        s_axes_known = true;
        ESP_LOGI(TAG, "controller reports transposed axes, swapping");
    }
    if (s_swap_axes) {
        const int t = tx;
        tx = ty;
        ty = t;
    }

    if (tx < 0 || tx >= LCD_H_RES || ty < 0 || ty >= LCD_V_RES) {
        return false;
    }
    s_axes_known = true;

    if (!s_was_down) {
        ESP_LOGI(TAG, "touch at x=%d y=%d", tx, ty);
    }
    s_was_down = true;
    *x = tx;
    *y = ty;
    return true;
}
