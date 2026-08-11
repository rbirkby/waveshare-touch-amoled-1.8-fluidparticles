// Central tuning header for the portable simulation and renderer.
//
// Everything here is board-independent: it describes the fluid and how it is
// drawn, not the hardware. Pins, I2C addresses and the IMU orientation live in
// platform/<target>/board_config.h, which this file pulls in at the bottom.
#pragma once

// ---------------------------------------------------------------------------
// Panel
// ---------------------------------------------------------------------------
#define LCD_H_RES 368
#define LCD_V_RES 448

// Height of one render band. 448 must be divisible by this, and it must be even
// because both panel controllers only accept even window coordinates.
#define TILE_H 32
#define TILE_COUNT (LCD_V_RES / TILE_H)

// ---------------------------------------------------------------------------
// Simulation box
// ---------------------------------------------------------------------------
// The box is exactly the display in X/Y. Z is the case depth: z = 0 is the
// glass (nearest the viewer), z = BOX_D_PX is the back of the case.
#define BOX_D_PX 96

// Particle count. Raise until the frame time reported on the serial log stops
// being comfortable.
#define PARTICLE_COUNT 600
#define PARTICLE_MAX 1600

// SPH smoothing radius in pixels. The solver works internally in units of H so
// that all the kernel constants stay close to 1.0.
#define SPH_H_INT 40
#define SPH_H_PX ((float)SPH_H_INT)
// Rest spacing of the particle lattice, as a fraction of H.
#define SPH_SPACING 0.55f
// Position-Based-Fluids solver iterations per step.
#define PBF_ITERATIONS 2
// Maximum neighbours cached per particle.
#define MAX_NEIGHBOURS 28

#define SIM_HZ 30
#define SIM_DT (1.0f / (float)SIM_HZ)

// Gravity felt by the fluid, in px/s^2 per 1 g of measured acceleration.
// Real water would be ~116000 px/s^2; that is far too fast to see on a 31 mm
// wide box, so the demo runs in slow motion.
#define GRAVITY_PX_PER_G 1800.0f
// Speed clamp in px/s. Must stay below SPH_H_PX * SIM_HZ or the neighbour
// search starts missing pairs.
#define SPEED_LIMIT_PX 900.0f

// Wall bounciness (0 = dead stop, 1 = perfect bounce) and tangential friction.
#define WALL_RESTITUTION 0.30f
#define WALL_FRICTION 0.92f

// XSPH artificial viscosity. Higher = thicker, more syrupy.
#define XSPH_VISCOSITY 0.22f
// Tensile instability correction (stops particles clumping into strings).
#define SCORR_K 0.0004f

// Largest positional correction one solver iteration may apply, in units of H.
// Velocity is derived from position change over dt, so this is what stops a
// single bad shove turning into a particle that rockets across the box.
#define MAX_CORRECTION 0.15f

// Exponential smoothing factor for the speed value that drives particle colour.
// 1.0 = instantaneous (strobes), lower = the tint follows sustained motion.
#define COLOUR_SMOOTHING 0.18f
#define SCORR_N 4
#define SCORR_DQ 0.25f
// Constraint force mixing, keeps lambda finite for lone particles.
#define CFM_EPSILON 1.0e-3f

// How strongly rotation of the device stirs the fluid (Coriolis + centrifugal).
#define ROTATION_SCALE 1.0f

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
// Radius of a particle at the glass, in pixels. Shrinks with depth.
#define PARTICLE_RADIUS_PX 6.0f
#define SPRITE_RADIUS_MAX 16
// Perspective focal length in pixels. Smaller = stronger depth foreshortening.
#define PERSPECTIVE_F 300.0f
// How much a particle at the back of the case is darkened (0..1).
#define DEPTH_DARKEN 0.55f

// Speed at which a particle reaches full "hot" colour, in px/s.
#define COLOUR_HOT_SPEED 520.0f

#define PALETTE_VEL_LEVELS 16
#define PALETTE_SHADE_LEVELS 16

// ---------------------------------------------------------------------------
// Touch interaction
// ---------------------------------------------------------------------------
// Touch repulsion. Radius is in pixels; strength is an acceleration in px/s^2
// applied at the centre of the touch and falling off linearly to zero at the
// radius. PUSH_BACK is the share of that which shoves particles away from the
// glass, so poking the screen dents the fluid instead of only sliding it.
#define TOUCH_RADIUS_PX 165.0f
#define TOUCH_STRENGTH  26000.0f
#define TOUCH_PUSH_BACK 0.45f

// Radius of the solid column the finger occupies, in pixels. Fluid is projected
// onto its surface, so this is a hard hole in the fluid rather than a force.
#define TOUCH_SOLID_PX 62.0f

// The board header supplies the pin map, the I2C addresses and the IMU axis
// mapping. It lives in platform/<target>/ and is the only part of the
// configuration that changes between the ESP32-S3 and the RP2350 boards.
#include "board_config.h"
