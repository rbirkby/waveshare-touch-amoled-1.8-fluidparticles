// 3D Position-Based Fluids (Macklin & Muller, SIGGRAPH 2013).
//
// Why PBF and not classic SPH? Classic weakly-compressible SPH needs a very
// small timestep to stay stable, which we cannot afford. PBF instead treats
// incompressibility as a constraint that is *projected* out by moving particles
// directly, which is unconditionally stable at a 1/60 s timestep. That is the
// difference between a demo that runs and one that explodes.
//
// Everything below works in units of the smoothing radius H, so H == 1 and all
// the kernel constants stay near 1.0. Positions are converted to pixels only
// when a snapshot is published to the renderer.

#include "fluid.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "hal.h"

static const char *TAG = "fluid";

// Box dimensions in H units.
#define BOX_X (LCD_H_RES / SPH_H_PX)
#define BOX_Y (LCD_V_RES / SPH_H_PX)
#define BOX_Z (BOX_D_PX / SPH_H_PX)
#define TOUCH_SOLID_R (TOUCH_SOLID_PX / SPH_H_PX)
#define WALL (SPH_SPACING * 0.5f)

// Uniform grid: one cell per smoothing radius, so a neighbour can only be in
// the 3x3x3 block of cells around a particle.
#define GRID_NX (LCD_H_RES / SPH_H_INT + 1)
#define GRID_NY (LCD_V_RES / SPH_H_INT + 1)
#define GRID_NZ (BOX_D_PX / SPH_H_INT + 1)
#define GRID_CELLS (GRID_NX * GRID_NY * GRID_NZ)

// Kernel normalisation constants for h = 1.
#define POLY6_C 1.5666814710f  // 315 / (64 pi)
#define SPIKY_C 14.3239448783f // 45 / pi

// Measured acceleration (m/s^2) to simulation acceleration (H/s^2).
#define ACC_TO_SIM (GRAVITY_PX_PER_G / (9.80665f * SPH_H_PX))
// TOUCH_STRENGTH is already an acceleration in px/s^2; the solver works in H.
#define PX_TO_SIM (1.0f / SPH_H_PX)
#define SPEED_LIMIT (SPEED_LIMIT_PX / SPH_H_PX)

static int N;

// Hot state, all in internal SRAM.
static float *px, *py, *pz;    // committed positions
static float *vx, *vy, *vz;    // velocities
static float *v0x, *v0y, *v0z; // velocity before the constraint solve
static float *q;               // predicted positions, packed xyz_ as stride 4
#define QX(i) q[((i) << 2) + 0]
#define QY(i) q[((i) << 2) + 1]
#define QZ(i) q[((i) << 2) + 2]
static float *lambda;
static float *dpx, *dpy, *dpz;
static float *scratch;
static float *scratch4;
static float *sdisp;          // temporally smoothed speed, for colour only
static uint16_t *cell_of;
static uint16_t *cell_start; // GRID_CELLS + 1 prefix sums
static uint16_t *reorder;
static uint16_t *neighbours; // N * MAX_NEIGHBOURS
static uint8_t *neigh_count;

// Cold state: the snapshot handed to the renderer lives in PSRAM.
static fluid_point_t *snapshot;
static hal_mutex_t *snapshot_lock;

static float rho0 = 1.0f;
static float scorr_denom = 1.0f;

static float g_ax, g_ay, g_az; // measured specific force
static float g_wx, g_wy, g_wz; // angular rate

static inline float kernel_poly6(float r2)
{
    const float t = 1.0f - r2;
    return POLY6_C * t * t * t;
}

// The LX7 FPU can multiply and fused-multiply-add in hardware, but it has no
// divide and no square root: both compile down to library calls that cost more
// than everything else in the inner loop combined. Two Newton-Raphson steps on
// the classic bit-trick seed give ~1e-6 relative accuracy using nothing but
// multiplies, which is far more precision than a fluid demo needs.
static inline float rsqrt_fast(float x)
{
    union {
        float f;
        int32_t i;
    } u = {.f = x};
    const float half = 0.5f * x;
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    y = y * (1.5f - half * y * y);
    y = y * (1.5f - half * y * y);
    return y;
}

static inline float sqrt_fast(float x)
{
    return x * rsqrt_fast(x);
}

static inline float frand(void)
{
    return (float)(hal_random_u32() & 0xFFFF) / 65535.0f;
}

// -------------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------------

// Rest density is whatever the poly6 kernel sums to for a particle sitting in
// the middle of a perfect lattice at the rest spacing. Deriving it instead of
// hard-coding a magic number means SPH_SPACING can be retuned freely.
static float compute_rest_density(void)
{
    float rho = 0.0f;
    const int k = (int)ceilf(1.0f / SPH_SPACING);
    for (int i = -k; i <= k; i++) {
        for (int j = -k; j <= k; j++) {
            for (int l = -k; l <= k; l++) {
                const float dx = i * SPH_SPACING;
                const float dy = j * SPH_SPACING;
                const float dz = l * SPH_SPACING;
                const float r2 = dx * dx + dy * dy + dz * dz;
                if (r2 < 1.0f) {
                    rho += kernel_poly6(r2);
                }
            }
        }
    }
    return rho;
}

static void *alloc_internal(size_t bytes)
{
    void *p = hal_alloc_fast(bytes);
    if (p == NULL) {
        hal_log(TAG, "out of internal RAM (%u bytes)", (unsigned)bytes);
        abort();
    }
    return p;
}

// Every heavy phase is written as a particle range so it can be split across
// both cores. That was implemented and measured twice - once with the helper at
// the render task's priority, once below it - and neither worked. The renderer
// is not idling on DMA as much as it looks; it genuinely needs core 0. At equal
// priority the helper stole frames (render 50 -> 28 fps), and at lower priority
// it starved, so the sim task simply blocked waiting for it (27 -> 48 ms/step).
// The chip is CPU-bound as a whole rather than core-bound, so the ranges all run
// on the sim core and the renderer keeps core 0 to itself. The signatures stay
// because re-testing the idea costs one line.
typedef void (*range_fn_t)(int i0, int i1);

static inline void parallel_for(range_fn_t fn)
{
    fn(0, N);
}

hal_err_t fluid_init(void)
{
    N = PARTICLE_COUNT;
    if (N > PARTICLE_MAX) {
        N = PARTICLE_MAX;
    }

    const size_t f = N * sizeof(float);
    float **vecs[] = {&px, &py, &pz, &vx, &vy, &vz, &v0x, &v0y, &v0z,
                      &lambda, &dpx, &dpy, &dpz, &scratch};
    for (size_t i = 0; i < sizeof(vecs) / sizeof(vecs[0]); i++) {
        *vecs[i] = alloc_internal(f);
    }
    q = alloc_internal(f * 4);
    scratch4 = alloc_internal(f * 4);
    sdisp = alloc_internal(f);
    cell_of = alloc_internal(N * sizeof(uint16_t));
    reorder = alloc_internal(N * sizeof(uint16_t));
    cell_start = alloc_internal((GRID_CELLS + 1) * sizeof(uint16_t));
    neighbours = alloc_internal((size_t)N * MAX_NEIGHBOURS * sizeof(uint16_t));
    neigh_count = alloc_internal(N);

    snapshot = hal_alloc_bulk(N * sizeof(fluid_point_t));
    if (snapshot == NULL) {
        snapshot = alloc_internal(N * sizeof(fluid_point_t));
    }
    snapshot_lock = hal_mutex_create();
    if (!snapshot_lock) {
        return HAL_FAIL;
    }

    rho0 = compute_rest_density();
    scorr_denom = kernel_poly6(SCORR_DQ * SCORR_DQ);

    hal_log(TAG, "%d particles, box %.2f x %.2f x %.2f H, grid %dx%dx%d, rho0 %.2f",
             N, BOX_X, BOX_Y, BOX_Z, GRID_NX, GRID_NY, GRID_NZ, rho0);
    fluid_reset();
    return HAL_OK;
}

void fluid_reset(void)
{
    // Drop a lattice of fluid into the lower part of the box, with a touch of
    // jitter so the first frame is not a perfectly symmetric crystal.
    memset(sdisp, 0, (size_t)N * sizeof(float));
    const float s = SPH_SPACING;
    const int nx = (int)((BOX_X - 2.0f * WALL) / s);
    const int nz = (int)((BOX_Z - 2.0f * WALL) / s);
    const int per_layer = (nx > 0 && nz > 0) ? nx * nz : 1;
    const int layers = (N + per_layer - 1) / per_layer;

    const float x0 = (BOX_X - (nx - 1) * s) * 0.5f;
    const float z0 = (BOX_Z - (nz - 1) * s) * 0.5f;
    const float y0 = BOX_Y - WALL - s * 0.5f;

    for (int i = 0; i < N; i++) {
        const int layer = i / per_layer;
        const int rem = i % per_layer;
        px[i] = x0 + (rem % nx) * s + (frand() - 0.5f) * s * 0.15f;
        pz[i] = z0 + (rem / nx) * s + (frand() - 0.5f) * s * 0.15f;
        py[i] = y0 - (layers - 1 - layer) * s + (frand() - 0.5f) * s * 0.15f;
        vx[i] = vy[i] = vz[i] = 0.0f;
    }
    for (int i = 0; i < N; i++) {
        QX(i) = px[i];
        QY(i) = py[i];
        QZ(i) = pz[i];
    }
}

int fluid_count(void)
{
    return N;
}

void fluid_set_motion(float ax, float ay, float az, float gx, float gy, float gz)
{
    g_ax = ax;
    g_ay = ay;
    g_az = az;
    g_wx = gx;
    g_wy = gy;
    g_wz = gz;
}

// -------------------------------------------------------------------------
// Neighbour search
// -------------------------------------------------------------------------

static inline int cell_index(float x, float y, float z)
{
    int cx = (int)x;
    int cy = (int)y;
    int cz = (int)z;
    if (cx < 0) cx = 0; else if (cx >= GRID_NX) cx = GRID_NX - 1;
    if (cy < 0) cy = 0; else if (cy >= GRID_NY) cy = GRID_NY - 1;
    if (cz < 0) cz = 0; else if (cz >= GRID_NZ) cz = GRID_NZ - 1;
    return cx + GRID_NX * (cy + GRID_NY * cz);
}

static void permute(float *arr, const uint16_t *order)
{
    for (int i = 0; i < N; i++) {
        scratch[i] = arr[order[i]];
    }
    memcpy(arr, scratch, N * sizeof(float));
}

static void permute_packed(const uint16_t *order)
{
    for (int i = 0; i < N; i++) {
        const int src = order[i] << 2;
        const int dst = i << 2;
        scratch4[dst + 0] = q[src + 0];
        scratch4[dst + 1] = q[src + 1];
        scratch4[dst + 2] = q[src + 2];
    }
    memcpy(q, scratch4, (size_t)N * 4 * sizeof(float));
}

// Counting-sorts every particle into grid order. Sorting the actual arrays (as
// opposed to just an index list) means the inner neighbour loops walk memory
// almost linearly, which matters a lot on a CPU with a small cache.
static void build_grid(void)
{
    memset(cell_start, 0, (GRID_CELLS + 1) * sizeof(uint16_t));
    for (int i = 0; i < N; i++) {
        const int c = cell_index(QX(i), QY(i), QZ(i));
        cell_of[i] = (uint16_t)c;
        cell_start[c + 1]++;
    }
    for (int c = 0; c < GRID_CELLS; c++) {
        cell_start[c + 1] += cell_start[c];
    }
    // cell_start is now the prefix sum; fill reorder using a moving cursor.
    static uint16_t cursor[GRID_CELLS];
    memcpy(cursor, cell_start, GRID_CELLS * sizeof(uint16_t));
    for (int i = 0; i < N; i++) {
        reorder[cursor[cell_of[i]]++] = (uint16_t)i;
    }

    permute(px, reorder);
    permute(py, reorder);
    permute(pz, reorder);
    permute(vx, reorder);
    permute(vy, reorder);
    permute(vz, reorder);
    permute(v0x, reorder);
    permute(v0y, reorder);
    permute(v0z, reorder);
    permute_packed(reorder);
}

// Caches the neighbour list once per step. All the solver passes then reuse it,
// which turns 5 expensive 27-cell searches into 1.
static void build_neighbours(int i0, int i1)
{
    for (int i = i0; i < i1; i++) {
        const float xi = QX(i), yi = QY(i), zi = QZ(i);
        int cx = (int)xi, cy = (int)yi, cz = (int)zi;
        if (cx < 0) cx = 0; else if (cx >= GRID_NX) cx = GRID_NX - 1;
        if (cy < 0) cy = 0; else if (cy >= GRID_NY) cy = GRID_NY - 1;
        if (cz < 0) cz = 0; else if (cz >= GRID_NZ) cz = GRID_NZ - 1;

        const int xlo = (cx > 0) ? cx - 1 : 0;
        const int xhi = (cx < GRID_NX - 1) ? cx + 1 : GRID_NX - 1;
        const int ylo = (cy > 0) ? cy - 1 : 0;
        const int yhi = (cy < GRID_NY - 1) ? cy + 1 : GRID_NY - 1;
        const int zlo = (cz > 0) ? cz - 1 : 0;
        const int zhi = (cz < GRID_NZ - 1) ? cz + 1 : GRID_NZ - 1;

        uint16_t *out = &neighbours[(size_t)i * MAX_NEIGHBOURS];
        float d2s[MAX_NEIGHBOURS];
        int n = 0;
        int worst = 0;          // slot holding the farthest kept neighbour
        float worst_d2 = 0.0f;
        for (int z = zlo; z <= zhi; z++) {
            for (int y = ylo; y <= yhi; y++) {
                // Cells are contiguous along x, so the three x cells collapse
                // into a single index range.
                const int base = GRID_NX * (y + GRID_NY * z);
                const int from = cell_start[base + xlo];
                const int to = cell_start[base + xhi + 1];
                for (int j = from; j < to; j++) {
                    if (j == i) {
                        continue;
                    }
                    const float dx = xi - QX(j);
                    const float dy = yi - QY(j);
                    const float dz = zi - QZ(j);
                    const float d2 = dx * dx + dy * dy + dz * dz;
                    if (d2 >= 1.0f) {
                        continue;
                    }
                    if (n < MAX_NEIGHBOURS) {
                        out[n] = (uint16_t)j;
                        d2s[n] = d2;
                        if (d2 > worst_d2) {
                            worst_d2 = d2;
                            worst = n;
                        }
                        n++;
                    } else if (d2 < worst_d2) {
                        // Full: evict the farthest so the list always holds the
                        // nearest neighbours, which dominate the density kernel.
                        out[worst] = (uint16_t)j;
                        d2s[worst] = d2;
                        worst_d2 = 0.0f;
                        for (int k = 0; k < MAX_NEIGHBOURS; k++) {
                            if (d2s[k] > worst_d2) {
                                worst_d2 = d2s[k];
                                worst = k;
                            }
                        }
                    }
                }
            }
        }
        neigh_count[i] = (uint8_t)n;
    }
}

// -------------------------------------------------------------------------
// Solver
// -------------------------------------------------------------------------

static volatile bool touch_on;
static volatile float touch_x, touch_y;

void fluid_set_touch(bool active, float x_px, float y_px)
{
    touch_x = x_px / SPH_H_PX;
    touch_y = y_px / SPH_H_PX;
    touch_on = active;
}

// The finger is treated as a solid column punched through the depth of the box,
// and this is what actually makes touch feel strong.
//
// Two things were wrong with the obvious approaches. A pure force field barely
// works, because the density solver exists precisely to cancel out anything
// that squashes particles together - push a blob of incompressible fluid and it
// just presses back, giving a shallow dent. So the finger has to be a
// *positional* constraint, exactly like the walls: particles are projected onto
// its surface, the solver has to respect it, and because velocity is derived
// from how far each particle moved, the projection generates the outward jet
// for free.
//
// The second problem was shape. Modelling the finger as a hemisphere sitting on
// the glass is the physically honest choice, but the case is only 96 px deep and
// the fluid piles up against whichever wall gravity points at - so with the
// board flat on a desk the fluid sits at the back and a shallow dome at the
// glass misses it entirely. A column spanning the full depth always hits, and
// reads as a rod pushed through the fluid.
static inline void clamp_to_box(float *x, float *y, float *z)
{
    if (touch_on) {
        const float dx = *x - touch_x;
        const float dy = *y - touch_y;
        const float d2 = dx * dx + dy * dy;
        if (d2 < TOUCH_SOLID_R * TOUCH_SOLID_R) {
            if (d2 > 1e-6f) {
                const float k = TOUCH_SOLID_R * rsqrt_fast(d2);
                *x = touch_x + dx * k;
                *y = touch_y + dy * k;
            } else {
                // Dead centre has no outward direction; nudge it off-axis and
                // let the next step resolve it properly.
                *x = touch_x + TOUCH_SOLID_R;
            }
        }
    }
    // Walls are applied last so a particle shoved by the finger can never be
    // pushed out through the side of the box.
    if (*x < WALL) *x = WALL; else if (*x > BOX_X - WALL) *x = BOX_X - WALL;
    if (*y < WALL) *y = WALL; else if (*y > BOX_Y - WALL) *y = BOX_Y - WALL;
    if (*z < WALL) *z = WALL; else if (*z > BOX_Z - WALL) *z = BOX_Z - WALL;
}

static void integrate_external(void)
{
    // The accelerometer measures specific force, which already contains both
    // gravity *and* whatever linear acceleration you apply by shaking. The
    // apparent gravity inside the box is simply its negation, so tilting and
    // shaking both come out of the same two lines. No sensor fusion needed.
    const float bx = -g_ax * ACC_TO_SIM;
    const float by = -g_ay * ACC_TO_SIM;
    const float bz = -g_az * ACC_TO_SIM;

    const float wx = g_wx * ROTATION_SCALE;
    const float wy = g_wy * ROTATION_SCALE;
    const float wz = g_wz * ROTATION_SCALE;
    const bool spinning = (wx * wx + wy * wy + wz * wz) > 1e-4f;

    const float cx = BOX_X * 0.5f, cy = BOX_Y * 0.5f, cz = BOX_Z * 0.5f;
    const float dt = SIM_DT;

    // The finger is a moving obstacle at the glass, so the push is strongest at
    // z = 0 and fades with depth as well as with radial distance.
    const bool poking = touch_on;
    const float tx = touch_x, ty = touch_y;
    const float trad = TOUCH_RADIUS_PX / SPH_H_PX;
    const float inv_trad = 1.0f / trad;
    const float tstr = TOUCH_STRENGTH * PX_TO_SIM;

    for (int i = 0; i < N; i++) {
        float ax = bx, ay = by, az = bz;

        if (spinning) {
            // Rotating reference frame: Coriolis (-2 w x v) makes the fluid
            // curl when you spin the board, centrifugal (-w x (w x r)) throws
            // it outwards.
            const float rx = px[i] - cx, ry = py[i] - cy, rz = pz[i] - cz;
            const float vxi = vx[i], vyi = vy[i], vzi = vz[i];

            ax -= 2.0f * (wy * vzi - wz * vyi);
            ay -= 2.0f * (wz * vxi - wx * vzi);
            az -= 2.0f * (wx * vyi - wy * vxi);

            const float sx = wy * rz - wz * ry;
            const float sy = wz * rx - wx * rz;
            const float sz = wx * ry - wy * rx;
            ax -= wy * sz - wz * sy;
            ay -= wz * sx - wx * sz;
            az -= wx * sy - wy * sx;
        }

        if (poking) {
            const float dx = px[i] - tx;
            const float dy = py[i] - ty;
            const float d2 = dx * dx + dy * dy;
            if (d2 < trad * trad) {
                // Linear falloff to zero at the radius, so there is no
                // discontinuity at the edge of the influence sphere.
                const float inv_d = (d2 > 1e-6f) ? rsqrt_fast(d2) : 0.0f;
                const float fall = 1.0f - d2 * inv_d * inv_trad;
                const float depth = 1.0f - pz[i] * (1.0f / BOX_Z);
                const float k = tstr * fall * fall * depth;
                ax += dx * inv_d * k;
                ay += dy * inv_d * k;
                az += TOUCH_PUSH_BACK * k;
            }
        }

        float nvx = vx[i] + ax * dt;
        float nvy = vy[i] + ay * dt;
        float nvz = vz[i] + az * dt;

        const float sp2 = nvx * nvx + nvy * nvy + nvz * nvz;
        if (sp2 > SPEED_LIMIT * SPEED_LIMIT) {
            const float k = SPEED_LIMIT * rsqrt_fast(sp2);
            nvx *= k;
            nvy *= k;
            nvz *= k;
        }

        vx[i] = v0x[i] = nvx;
        vy[i] = v0y[i] = nvy;
        vz[i] = v0z[i] = nvz;

        float x = px[i] + nvx * dt;
        float y = py[i] + nvy * dt;
        float z = pz[i] + nvz * dt;
        clamp_to_box(&x, &y, &z);
        QX(i) = x;
        QY(i) = y;
        QZ(i) = z;
    }
}

static void solve_densities(int i0, int i1)
{
    const float inv_rho0 = 1.0f / rho0;
    for (int i = i0; i < i1; i++) {
        const float xi = QX(i), yi = QY(i), zi = QZ(i);
        const uint16_t *nb = &neighbours[(size_t)i * MAX_NEIGHBOURS];
        const int n = neigh_count[i];

        float rho = POLY6_C; // self contribution, W(0)
        // Accumulated gradient of the density constraint w.r.t. this particle,
        // plus the sum of squared gradients w.r.t. every neighbour.
        float gx = 0.0f, gy = 0.0f, gz = 0.0f, sum_sq = 0.0f;

        for (int k = 0; k < n; k++) {
            const int j = nb[k];
            const float dx = xi - QX(j);
            const float dy = yi - QY(j);
            const float dz = zi - QZ(j);
            const float r2 = dx * dx + dy * dy + dz * dz;
            if (r2 >= 1.0f || r2 < 1e-9f) {
                continue;
            }
            rho += kernel_poly6(r2);

            const float inv_r = rsqrt_fast(r2);
            const float t = 1.0f - r2 * inv_r;
            const float w = -SPIKY_C * t * t * inv_r * inv_rho0;
            const float ex = w * dx, ey = w * dy, ez = w * dz;
            gx += ex;
            gy += ey;
            gz += ez;
            sum_sq += ex * ex + ey * ey + ez * ez;
        }

        // PBF treats density as an *inequality* constraint: rho <= rho0. Solving
        // it in the other direction too means any particle with a deficient
        // neighbourhood - i.e. every particle on the surface - gets sucked
        // violently inward, which is exactly the jitter that shows up as
        // flickering high-velocity specks. Clamping here is what makes the
        // surface behave like a surface.
        float c = rho * inv_rho0 - 1.0f;
        if (c < 0.0f) {
            c = 0.0f;
        }
        sum_sq += gx * gx + gy * gy + gz * gz;
        lambda[i] = -c / (sum_sq + CFM_EPSILON);
    }
}

static void solve_positions(int i0, int i1)
{
    const float inv_rho0 = 1.0f / rho0;
    const float inv_scorr = 1.0f / scorr_denom;
    for (int i = i0; i < i1; i++) {
        const float xi = QX(i), yi = QY(i), zi = QZ(i);
        const float li = lambda[i];
        const uint16_t *nb = &neighbours[(size_t)i * MAX_NEIGHBOURS];
        const int n = neigh_count[i];

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        for (int k = 0; k < n; k++) {
            const int j = nb[k];
            const float dx = xi - QX(j);
            const float dy = yi - QY(j);
            const float dz = zi - QZ(j);
            const float r2 = dx * dx + dy * dy + dz * dz;
            if (r2 >= 1.0f || r2 < 1e-9f) {
                continue;
            }
            const float inv_r = rsqrt_fast(r2);
            const float t = 1.0f - r2 * inv_r;

            // Artificial pressure. Without it particles pull themselves into
            // strings wherever the neighbourhood is sparse (tensile instability).
            const float ratio = kernel_poly6(r2) * inv_scorr;
            float scorr = ratio;
            for (int e = 1; e < SCORR_N; e++) {
                scorr *= ratio;
            }
            scorr *= -SCORR_K;

            const float w = -SPIKY_C * t * t * inv_r * (li + lambda[j] + scorr);
            ax += w * dx;
            ay += w * dy;
            az += w * dz;
        }
        ax *= inv_rho0;
        ay *= inv_rho0;
        az *= inv_rho0;

        // Velocity is derived from how far the solver moved a particle, divided
        // by dt - so at 30 Hz every unit of positional correction is multiplied
        // by 30 on its way into the velocity field. One bad shove becomes a
        // particle that shoots across the box. Cap the correction per iteration.
        const float m2 = ax * ax + ay * ay + az * az;
        if (m2 > MAX_CORRECTION * MAX_CORRECTION) {
            const float k = MAX_CORRECTION * rsqrt_fast(m2);
            ax *= k;
            ay *= k;
            az *= k;
        }
        dpx[i] = ax;
        dpy[i] = ay;
        dpz[i] = az;
    }

}

static void apply_positions(int i0, int i1)
{
    for (int i = i0; i < i1; i++) {
        float x = QX(i) + dpx[i];
        float y = QY(i) + dpy[i];
        float z = QZ(i) + dpz[i];
        clamp_to_box(&x, &y, &z);
        QX(i) = x;
        QY(i) = y;
        QZ(i) = z;
    }
}

// Derives velocity from how far the solver actually moved each particle, then
// smooths it against the neighbourhood (XSPH) so the fluid looks cohesive
// rather than like a bag of bouncing marbles.
static void derive_velocity(int i0, int i1)
{
    const float inv_dt = 1.0f / SIM_DT;
    for (int i = i0; i < i1; i++) {
        vx[i] = (QX(i) - px[i]) * inv_dt;
        vy[i] = (QY(i) - py[i]) * inv_dt;
        vz[i] = (QZ(i) - pz[i]) * inv_dt;
    }
}

static void xsph(int i0, int i1)
{
    const float inv_rho0 = 1.0f / rho0;
    for (int i = i0; i < i1; i++) {
        const float xi = QX(i), yi = QY(i), zi = QZ(i);
        const float vxi = vx[i], vyi = vy[i], vzi = vz[i];
        const uint16_t *nb = &neighbours[(size_t)i * MAX_NEIGHBOURS];
        const int n = neigh_count[i];

        float sx = 0.0f, sy = 0.0f, sz = 0.0f;
        for (int k = 0; k < n; k++) {
            const int j = nb[k];
            const float dx = xi - QX(j);
            const float dy = yi - QY(j);
            const float dz = zi - QZ(j);
            const float r2 = dx * dx + dy * dy + dz * dz;
            if (r2 >= 1.0f) {
                continue;
            }
            const float w = kernel_poly6(r2);
            sx += (vx[j] - vxi) * w;
            sy += (vy[j] - vyi) * w;
            sz += (vz[j] - vzi) * w;
        }
        dpx[i] = vxi + XSPH_VISCOSITY * sx * inv_rho0;
        dpy[i] = vyi + XSPH_VISCOSITY * sy * inv_rho0;
        dpz[i] = vzi + XSPH_VISCOSITY * sz * inv_rho0;
    }
}

static void finalise(void)
{
    parallel_for(derive_velocity);
    parallel_for(xsph);

    const float lo = WALL + 1e-3f;
    for (int i = 0; i < N; i++) {
        float nvx = dpx[i], nvy = dpy[i], nvz = dpz[i];
        const float x = QX(i), y = QY(i), z = QZ(i);

        // The position clamp above already absorbed all of the wall-normal
        // velocity. Put a fraction of it back as a bounce and shave the
        // tangential component to fake friction.
        if (x <= lo || x >= BOX_X - lo) {
            nvx = -WALL_RESTITUTION * v0x[i];
            nvy *= WALL_FRICTION;
            nvz *= WALL_FRICTION;
        }
        if (y <= lo || y >= BOX_Y - lo) {
            nvy = -WALL_RESTITUTION * v0y[i];
            nvx *= WALL_FRICTION;
            nvz *= WALL_FRICTION;
        }
        if (z <= lo || z >= BOX_Z - lo) {
            nvz = -WALL_RESTITUTION * v0z[i];
            nvx *= WALL_FRICTION;
            nvy *= WALL_FRICTION;
        }

        vx[i] = nvx;
        vy[i] = nvy;
        vz[i] = nvz;
        px[i] = x;
        py[i] = y;
        pz[i] = z;
    }
}

static void publish(void)
{
    const float inv_hot = SPH_H_PX / COLOUR_HOT_SPEED;
    hal_mutex_lock(snapshot_lock);
    for (int i = 0; i < N; i++) {
        snapshot[i].x = px[i] * SPH_H_PX;
        snapshot[i].y = py[i] * SPH_H_PX;
        snapshot[i].z = pz[i] * SPH_H_PX;
        const float s2 = vx[i] * vx[i] + vy[i] * vy[i] + vz[i] * vz[i];
        float s = (s2 > 1e-12f) ? sqrt_fast(s2) * inv_hot : 0.0f;
        if (s > 1.0f) {
            s = 1.0f;
        }
        // Instantaneous speed is a twitchy signal, and mapping it straight to
        // hue makes the colours strobe even when the flow itself is smooth.
        // A short exponential average makes the tint follow sustained motion.
        sdisp[i] += (s - sdisp[i]) * COLOUR_SMOOTHING;
        snapshot[i].speed01 = sdisp[i];
    }
    hal_mutex_unlock(snapshot_lock);
}

uint32_t fluid_phase_us[5];

void fluid_step(void)
{
    uint64_t t0 = hal_time_us(), t1;
    integrate_external();
    t1 = hal_time_us(); fluid_phase_us[0] += (uint32_t)(t1 - t0); t0 = t1;
    build_grid();
    t1 = hal_time_us(); fluid_phase_us[1] += (uint32_t)(t1 - t0); t0 = t1;
    parallel_for(build_neighbours);
    t1 = hal_time_us(); fluid_phase_us[2] += (uint32_t)(t1 - t0); t0 = t1;
    for (int it = 0; it < PBF_ITERATIONS; it++) {
        parallel_for(solve_densities);
        parallel_for(solve_positions);
        parallel_for(apply_positions);
    }
    t1 = hal_time_us(); fluid_phase_us[3] += (uint32_t)(t1 - t0); t0 = t1;
    finalise();
    publish();
    t1 = hal_time_us(); fluid_phase_us[4] += (uint32_t)(t1 - t0);
}

int fluid_snapshot(fluid_point_t *dst)
{
    hal_mutex_lock(snapshot_lock);
    memcpy(dst, snapshot, N * sizeof(fluid_point_t));
    hal_mutex_unlock(snapshot_lock);
    return N;
}
