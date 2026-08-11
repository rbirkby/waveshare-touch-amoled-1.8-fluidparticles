// ESP32-S3 internal helpers, shared between this board's drivers.
// The portable code never includes this; it uses src/drivers.h.
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

// Brings up the single shared I2C bus used by the PMIC, IMU and touch panel.
esp_err_t board_i2c_init(void);

i2c_master_bus_handle_t board_i2c_bus(void);

// Adds a device to the shared bus, returning NULL if it does not answer.
i2c_master_dev_handle_t board_i2c_add(uint8_t addr);

esp_err_t board_reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);
esp_err_t board_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *dst, size_t len);

// Releases the panel and touch reset lines held by the TCA9554 IO expander.
// Must run after board_i2c_init() and before display_init().
esp_err_t touch_release_reset(void);
