// Board bring-up for the RP2350-Touch-AMOLED-1.8.
//
// Much simpler than the ESP32-S3 sibling: there is no IO expander holding
// things in reset, so this only has to raise the clock, power the panel rail,
// open I2C and release the two reset lines.

#include "board.h"

#include "board_config.h"
#include "drivers.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hal.h"
#include "pico/stdlib.h"

static const char *TAG = "board";

void board_i2c_init(void)
{
    i2c_init(BOARD_I2C_INST, BOARD_I2C_HZ);
    gpio_set_function(BOARD_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(BOARD_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(BOARD_I2C_SDA);
    gpio_pull_up(BOARD_I2C_SCL);
}

bool board_i2c_probe(uint8_t addr)
{
    uint8_t dummy;
    return i2c_read_blocking_until(BOARD_I2C_INST, addr, &dummy, 1, false,
                                   make_timeout_time_ms(5)) >= 0;
}

bool board_reg_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_write_blocking_until(BOARD_I2C_INST, addr, buf, 2, false,
                                    make_timeout_time_ms(10)) == 2;
}

bool board_reg_read(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len)
{
    if (i2c_write_blocking_until(BOARD_I2C_INST, addr, &reg, 1, true,
                                 make_timeout_time_ms(10)) != 1) {
        return false;
    }
    return i2c_read_blocking_until(BOARD_I2C_INST, addr, dst, len, false,
                                   make_timeout_time_ms(10)) == (int)len;
}

hal_err_t board_init(void)
{
    // The solver is CPU-bound, so take the clock above the 150 MHz default.
    // set_sys_clock_khz returns false if the PLL cannot make the frequency, in
    // which case the SDK default stands and everything still works, just slower.
    if (!set_sys_clock_khz(SYS_CLOCK_KHZ, false)) {
        hal_log(TAG, "could not set %d kHz, staying at default", SYS_CLOCK_KHZ);
    }

    // The panel rail must be live before the SH8601 will answer.
    gpio_init(LCD_PIN_PWR_EN);
    gpio_set_dir(LCD_PIN_PWR_EN, GPIO_OUT);
    gpio_put(LCD_PIN_PWR_EN, 1);

    board_i2c_init();
    hal_log(TAG, "clock %u kHz, I2C%d on sda=%d scl=%d",
            (unsigned)(clock_get_hz(clk_sys) / 1000),
            BOARD_I2C_INST == i2c0 ? 0 : 1, BOARD_I2C_SDA, BOARD_I2C_SCL);
    return HAL_OK;
}
