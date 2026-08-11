// Board wiring for the Waveshare ESP32-S3-Touch-AMOLED-1.8.
//
// Every value here was verified on the physical board, either by probing it or
// by reading Waveshare's own BSP. See README part 1 for how each was found.
#pragma once

#include "driver/i2c_master.h"

// Panel controller: CO5300, 368x448 RGB565 over the ESP32-S3 hardware QSPI.
#define BOARD_NAME "ESP32-S3-Touch-AMOLED-1.8"

// ---------------------------------------------------------------------------
// IMU axis mapping
// ---------------------------------------------------------------------------
// The QMI8658 is soldered in some fixed orientation relative to the screen.
// IMU axis mapping. The QMI8658 is mounted rotated 90 degrees relative to the
// panel, so the raw axes need re-labelling before the solver sees them.
//
// Two independent facts pin this down:
//   1. Measured here: flat with the screen up reads accel = (~0, ~0, -9.8), so
//      the IMU +Z axis points *into* the screen -- the same direction as the
//      simulation's +Z (towards the back of the case). Z needs no change.
//   2. Waveshare's own auto-rotation demo (arduino-v2/13_LVGL_Widgets) switches
//      the display to its default upright orientation when accel.x > +0.8 g. An
//      accelerometer at rest reads the "up" direction, so +X on the IMU points
//      at the *top* of the screen.
//
// From (2), IMU +X = screen up = simulation -Y, and IMU +Y = screen right =
// simulation +X. That is a proper rotation (determinant +1), so the gyro's
// pseudo-vector transforms with exactly the same mapping.
//
//     sim x = +imu y        sim y = -imu x        sim z = +imu z
//
// If the fluid ever falls sideways again, this block is the only thing to edit.
#define IMU_SIGN_X (+1.0f)
#define IMU_SIGN_Y (-1.0f)
#define IMU_SIGN_Z (+1.0f)
// Set to 1 if screen X should follow the IMU Y axis instead of X.
#define IMU_SWAP_XY 1

// ---------------------------------------------------------------------------
// Board wiring (verified against the Waveshare BSP)
// ---------------------------------------------------------------------------
#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_I2C_SDA 15
#define BOARD_I2C_SCL 14
#define BOARD_I2C_HZ 400000

#define LCD_SPI_HOST SPI2_HOST
#define LCD_PIN_CS 12
#define LCD_PIN_PCLK 11
#define LCD_PIN_D0 4
#define LCD_PIN_D1 5
#define LCD_PIN_D2 6
#define LCD_PIN_D3 7

#define ADDR_AXP2101 0x34
#define ADDR_QMI8658 0x6B
#define ADDR_QMI8658_ALT 0x6A
#define ADDR_TOUCH_CST816 0x15
#define ADDR_TOUCH_FT3168 0x38
#define ADDR_IO_EXPANDER  0x20

// TCA9554 IO expander. It gates the panel reset, the touch reset and the SD
// card chip select, all of which power up asserted.
#define TCA9554_REG_OUTPUT 0x01
#define TCA9554_REG_CONFIG 0x03
#define TCA9554_LCD_RST    (1 << 0)
#define TCA9554_DSI_PWR_EN (1 << 1)
#define TCA9554_TOUCH_RST  (1 << 2)
#define TCA9554_SD_CS      (1 << 7)
#define TCA9554_OUTPUT_MASK \
    (TCA9554_LCD_RST | TCA9554_DSI_PWR_EN | TCA9554_TOUCH_RST | TCA9554_SD_CS)

// FocalTech-style contact registers, shared by the CST816 and the FT3168.
#define TOUCH_REG_STATUS 0x02
#define TOUCH_REG_CHIP_ID 0xA3

// Touch controller interrupt, active low while a finger is down.
#define TOUCH_PIN_INT 21

// Touch repulsion. Radius is in pixels; strength is an acceleration in px/s^2
// applied at the centre of the touch and falling off linearly to zero at the
// radius. PUSH_BACK is the share of that which shoves particles away from the
// glass, so poking the screen dents the fluid instead of only sliding it.
#define TOUCH_RADIUS_PX 165.0f
#define TOUCH_STRENGTH  26000.0f
#define TOUCH_PUSH_BACK 0.45f

// Radius of the solid hemisphere the finger occupies, in pixels. Fluid is
// projected onto its surface, so this is a hard hole in the fluid rather than a
// force. Keep it comfortably under BOX_D_PX or the sphere pokes out the back.
#define TOUCH_SOLID_PX 62.0f
