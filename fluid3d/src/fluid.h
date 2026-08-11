#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

// One particle as handed to the renderer: position in pixels inside the box
// (x: 0..368, y: 0..448, z: 0..BOX_D_PX) plus a normalised speed for colouring.
typedef struct {
    float x, y, z;
    float speed01;
} fluid_point_t;

hal_err_t fluid_init(void);

// Refills the box with a settled block of fluid.
void fluid_reset(void);

// Latest inertial state, in m/s^2 and rad/s, already in simulation axes.
void fluid_set_motion(float ax, float ay, float az, float gx, float gy, float gz);

// Reports where the user is touching the glass, in panel pixels. Set active to
// false when no finger is down.
void fluid_set_touch(bool active, float x_px, float y_px);

// Advances the simulation by one fixed timestep and publishes a snapshot.
void fluid_step(void);

// Copies the most recent published snapshot. Returns the particle count.
int fluid_snapshot(fluid_point_t *dst);

int fluid_count(void);

// Per-phase timing accumulators in microseconds, for the stats log.
extern uint32_t fluid_phase_us[5];
