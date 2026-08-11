// FT3168 capacitive touch, for the RP2350 board.
//
// Simpler than the ESP32-S3 sibling: reset is a plain GPIO here rather than a
// bit on an IO expander, so the controller answers as soon as it is released.
//
// The controller sleeps when nothing is touching it and then refuses to
// acknowledge its own address, which fills the log with I2C errors if it is
// polled blindly. Its interrupt line is low for exactly as long as a finger is
// down, so that pin is used as a gate and I2C is only touched when there is
// something to read.

#include "drivers.h"

#include "board.h"
#include "board_config.h"
#include "config.h"
#include "hal.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

static const char *TAG = "touch";

static uint8_t s_addr;
static bool s_transposed;

static void touch_reset(void)
{
    gpio_init(TOUCH_PIN_RST);
    gpio_set_dir(TOUCH_PIN_RST, GPIO_OUT);
    gpio_put(TOUCH_PIN_RST, 1);
    sleep_ms(20);
    gpio_put(TOUCH_PIN_RST, 0);
    sleep_ms(20);
    gpio_put(TOUCH_PIN_RST, 1);
    sleep_ms(100);
}

hal_err_t touch_init(void)
{
    touch_reset();

    gpio_init(TOUCH_PIN_INT);
    gpio_set_dir(TOUCH_PIN_INT, GPIO_IN);
    gpio_pull_up(TOUCH_PIN_INT);

    s_addr = 0;
    if (board_i2c_probe(ADDR_TOUCH_FT3168)) {
        s_addr = ADDR_TOUCH_FT3168;
    } else if (board_i2c_probe(ADDR_TOUCH_CST816)) {
        s_addr = ADDR_TOUCH_CST816;
    }
    if (s_addr == 0) {
        hal_log(TAG, "no touch controller found");
        return HAL_FAIL;
    }

    uint8_t id = 0;
    board_reg_read(s_addr, TOUCH_REG_CHIP_ID, &id, 1);
    hal_log(TAG, "controller at 0x%02X, chip id 0x%02X", s_addr, id);
    return HAL_OK;
}

bool touch_read(int *x, int *y)
{
    if (s_addr == 0) {
        return false;
    }
    // Gate on the interrupt line: no finger, no I2C traffic.
    if (gpio_get(TOUCH_PIN_INT) != 0) {
        return false;
    }

    uint8_t buf[5];
    if (!board_reg_read(s_addr, TOUCH_REG_STATUS, buf, sizeof(buf))) {
        return false;
    }
    if ((buf[0] & 0x0F) == 0) {
        return false;
    }

    int px = ((buf[1] & 0x0F) << 8) | buf[2];
    int py = ((buf[3] & 0x0F) << 8) | buf[4];

    // Some panel variants report the axes the other way round. If a coordinate
    // only makes sense transposed, latch that and stay consistent afterwards.
    if (!s_transposed && (px >= LCD_H_RES && px < LCD_V_RES && py < LCD_H_RES)) {
        s_transposed = true;
        hal_log(TAG, "touch axes look transposed, swapping");
    }
    if (s_transposed) {
        const int t = px;
        px = py;
        py = t;
    }

    if (px < 0) px = 0; else if (px >= LCD_H_RES) px = LCD_H_RES - 1;
    if (py < 0) py = 0; else if (py >= LCD_V_RES) py = LCD_V_RES - 1;

    *x = px;
    *y = py;
    return true;
}
