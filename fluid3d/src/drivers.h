// Device driver contract.
//
// These four small interfaces are what the portable application needs from a
// board. Each platform under platform/<target>/ implements all of them; the
// code in src/ never learns which board it is running on.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Board bring-up
// ---------------------------------------------------------------------------
// Powers the rails, releases resets and opens the shared I2C bus. Called first,
// before any other driver. Everything board-specific and order-dependent (the
// ESP32-S3's IO expander dance, the RP2350's power-enable pin) hides in here.
hal_err_t board_init(void);

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
// The panel is identical on both boards (368x448 RGB565 AMOLED over a 4-bit
// QSPI bus) but the controller and the transport differ, so the renderer talks
// to it purely through this band-streaming interface. Neither board has enough
// RAM for a full framebuffer, so a frame is pushed as TILE_COUNT horizontal
// strips with two buffers ping-ponged through DMA.
hal_err_t display_init(void);

// Returns a DMA-capable band buffer whose previous transfer has completed.
// Blocks if every buffer is still in flight.
uint16_t *display_acquire_band(void);

// Hands the band back for transmission. `band` is the strip index,
// 0 .. TILE_COUNT-1. Returns immediately; the transfer runs on DMA.
void display_send_band(uint16_t *buf, int band);

// Waits for every queued transfer to complete.
void display_wait_idle(void);

// True when the panel expects pixels in big-endian byte order. The ESP32-S3's
// SPI peripheral needs the swap done in software; the RP2350's PIO program can
// shift them out in the right order for free.
bool display_needs_byte_swap(void);

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------
typedef struct {
    float ax, ay, az; // specific force, m/s^2, already mapped to sim axes
    float gx, gy, gz; // angular rate, rad/s, already mapped to sim axes
} sensors_sample_t;

hal_err_t sensors_init(void);

// Reads the IMU, applies the gyro bias measured at startup and the board's
// axis mapping, so the caller always receives simulation axes.
hal_err_t sensors_read(sensors_sample_t *out);

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------
// True exactly once per press of the user button (the one that is not BOOT).
// On the ESP32-S3 this is the PMIC's power key; on the RP2350 it is a plain
// GPIO. Both boards must keep their normal reboot path working.
bool button_pressed(void);

// ---------------------------------------------------------------------------
// Touch
// ---------------------------------------------------------------------------
hal_err_t touch_init(void);

// Returns true and fills x/y in panel pixels while a finger is down.
bool touch_read(int *x, int *y);

#ifdef __cplusplus
}
#endif
