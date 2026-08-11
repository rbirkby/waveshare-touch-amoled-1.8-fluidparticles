#include "board.h"

#include "drivers.h"

#include "config.h"
#include "esp_log.h"

static const char *TAG = "board";
static i2c_master_bus_handle_t s_bus;

esp_err_t board_i2c_init(void)
{
    if (s_bus != NULL) {
        return ESP_OK;
    }
    const i2c_master_bus_config_t cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA,
        .scl_io_num = BOARD_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &s_bus);
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return s_bus;
}

i2c_master_dev_handle_t board_i2c_add(uint8_t addr)
{
    if (s_bus == NULL || i2c_master_probe(s_bus, addr, 50) != ESP_OK) {
        ESP_LOGW(TAG, "no I2C device at 0x%02X", addr);
        return NULL;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = BOARD_I2C_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(s_bus, &cfg, &dev) != ESP_OK) {
        return NULL;
    }
    return dev;
}

esp_err_t board_reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

esp_err_t board_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, dst, len, 100);
}

// The portable entry point. Order matters: the IO expander holds the panel and
// touch controllers in reset at power-on, so the bus must exist first and the
// resets must be released before display_init() runs.
hal_err_t board_init(void)
{
    esp_err_t err = board_i2c_init();
    if (err != ESP_OK) {
        return HAL_FAIL;
    }
    touch_release_reset();
    return HAL_OK;
}
