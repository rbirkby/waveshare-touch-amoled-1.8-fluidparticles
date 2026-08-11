// RP2350 internal helpers, shared between this board's drivers.
// The portable code never includes this; it uses src/drivers.h.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Opens the shared I2C bus. Called by board_init().
void board_i2c_init(void);

// True if a device acknowledges its address.
bool board_i2c_probe(uint8_t addr);

// Single-register write, and a block read starting at a register.
bool board_reg_write(uint8_t addr, uint8_t reg, uint8_t val);
bool board_reg_read(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len);
