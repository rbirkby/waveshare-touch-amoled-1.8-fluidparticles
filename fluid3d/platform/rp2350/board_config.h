// Board wiring for the Waveshare RP2350-Touch-AMOLED-1.8.
//
// Same 368x448 AMOLED panel and the same FT3168 touch and QMI8658 IMU as the
// ESP32-S3 sibling board, but almost every wire moved and the plumbing is
// different in three ways worth knowing about:
//
//   1. There is no TCA9554 IO expander. Panel reset, panel power-enable and
//      touch reset are all plain GPIOs here, which makes bring-up much simpler.
//   2. The panel controller is an SH8601 rather than a CO5300. Both speak the
//      same QSPI framing (0x02 = register write, 0x32 = pixel stream) and the
//      same MIPI window commands, so only the init register list differs.
//   3. The RP2350 has no hardware QSPI for peripherals -- its QMI is wired to
//      flash -- so the 4-bit bus is synthesised with a PIO state machine.
#pragma once

#define BOARD_NAME "RP2350-Touch-AMOLED-1.8"

// ---------------------------------------------------------------------------
// Display: SH8601 over a PIO-driven 4-bit QSPI bus
// ---------------------------------------------------------------------------
#define LCD_PIN_CS 9
#define LCD_PIN_PCLK 10
#define LCD_PIN_D0 11
#define LCD_PIN_D1 12
#define LCD_PIN_D2 13
#define LCD_PIN_D3 14
#define LCD_PIN_RST 15
// Drives the panel's power rail. Must be high before the controller answers.
#define LCD_PIN_PWR_EN 17

// The panel's tearing-effect output. Not documented by Waveshare -- found by
// enabling TE (register 0x35) and scanning every free GPIO for a pulse, which
// turned up GPIO 16 toggling at ~60 Hz. Unused by this firmware, but it is a
// very handy liveness check: if the panel is powered and has accepted its
// initialisation, this pin ticks at the refresh rate.
#define LCD_PIN_TE 16

// Column offset of the visible area. The 1.75" sibling board needs 6; this
// panel starts at 0. If the image is shifted sideways, this is the knob.
#define LCD_X_GAP 0

// PIO clock divider. 1.0 runs the state machine at the full system clock, and
// the program spends two cycles per nibble, so the bus runs at sys_clk/2.
// PIO clock divider. The state machine takes two cycles per nibble, so the bus
// runs at sys_clk / (2 * divider) -- here 200 MHz / 4 = 50 MHz. Waveshare clock
// their board at 150 MHz with a divider of 1, i.e. a 75 MHz bus, so this stays
// comfortably inside what the panel is known to accept while leaving the CPU at
// 200 MHz for the solver. It is still far faster than the renderer can fill.
#define LCD_PIO_CLKDIV 2.0f

// ---------------------------------------------------------------------------
// I2C: the IMU, the touch controller, the PMIC and the RTC share one bus
// ---------------------------------------------------------------------------
#define BOARD_I2C_INST i2c1
#define BOARD_I2C_SDA 6
#define BOARD_I2C_SCL 7
#define BOARD_I2C_HZ 400000

#define ADDR_AXP2101 0x34
#define ADDR_QMI8658 0x6B
#define ADDR_QMI8658_ALT 0x6A
#define ADDR_TOUCH_FT3168 0x38
#define ADDR_TOUCH_CST816 0x15

// FocalTech-style contact registers, shared by the CST816 and the FT3168.
#define TOUCH_REG_STATUS 0x02
#define TOUCH_REG_CHIP_ID 0xA3

// Touch controller reset (direct GPIO here, not an IO expander) and its
// interrupt line, which is low while a finger is down.
#define TOUCH_PIN_RST 5
#define TOUCH_PIN_INT 4

// QMI8658 interrupt 1. Not used: the IMU is polled.
#define IMU_PIN_INT1 8

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------
// The user button is the AXP2101's power key, exactly as on the ESP32-S3 board.
// Waveshare's own examples define a "SYS_OUT" on GPIO 18 and treat it as a key,
// but probing this board shows GPIO 18 is driven low permanently and no GPIO at
// all floats like an idle active-low button, so it is a power-latch output and
// not a button. The PMIC path is the one that works, and it has the pleasant
// side effect of making button handling identical on both boards.
//
// The key is polled rather than claimed as an interrupt, which deliberately
// leaves the PMIC's own long-press power-off intact so reflashing always works.
// BOOT is handled by the bootrom and is invisible to the application, so the
// usual "hold BOOT while plugging in" route is untouched; picotool can also
// reboot the board over USB without pressing anything.
#define AXP2101_REG_INTEN2 0x41
#define AXP2101_REG_INTSTS2 0x49
#define AXP2101_PWRKEY_BITS 0x0F

// ---------------------------------------------------------------------------
// System clock
// ---------------------------------------------------------------------------
// The RP2350 is specified to 150 MHz; Waveshare's own 1.75" example runs at
// 200 MHz. The solver is the bottleneck, so the extra headroom is worth having.
#define SYS_CLOCK_KHZ (200 * 1000)

// ---------------------------------------------------------------------------
// IMU axis mapping
// ---------------------------------------------------------------------------
// Same reasoning as the ESP32-S3 board (see that board_config.h for the full
// derivation): the mapping re-labels the raw QMI8658 axes so the solver always
// receives "screen right / screen down / into the case".
//
//     sim x = +imu y        sim y = -imu x        sim z = +imu z
//
// The QMI8658 sits on a different PCB here, so this was re-checked on the
// hardware rather than assumed. If the fluid falls sideways, edit this block.
#define IMU_SIGN_X (+1.0f)
#define IMU_SIGN_Y (-1.0f)
#define IMU_SIGN_Z (+1.0f)
#define IMU_SWAP_XY 1
