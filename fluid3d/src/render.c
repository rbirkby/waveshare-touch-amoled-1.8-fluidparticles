// Software particle rasteriser.
//
// There is no GPU and not enough RAM for a full 330 KB framebuffer plus a back
// buffer, so the frame is produced one 368x32 band at a time straight into the
// buffer that DMA will send. Three tricks keep it cheap:
//
//   * Particles are bucketed by depth and drawn back-to-front (painter's
//     algorithm), so no depth buffer is needed.
//   * Each sprite is a precomputed set of horizontal spans, so the body of a
//     particle is a tight run of stores rather than a per-pixel circle test.
//   * Colour is a 16x16 lookup table indexed by (speed, brightness), so the
//     inner loop never does arithmetic on colour channels.

#include "render.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "drivers.h"
#include "fluid.h"
#include "hal.h"

static const char *TAG = "render";

// Set from display_needs_byte_swap() at init. The ESP32-S3 pushes RGB565 out
// of a byte-oriented SPI peripheral so the halves arrive reversed and have to
// be swapped in software; the RP2350's PIO program shifts nibbles in panel
// order, so there the band can simply be copied.
static bool s_swap;


#define SPRITE_MIN 2
#define SPRITE_SPAN (2 * SPRITE_RADIUS_MAX + 1)
#define Z_BUCKETS 32

typedef struct {
    int8_t xo[SPRITE_SPAN]; // opaque half-width per row, -1 for none
    int8_t xe[SPRITE_SPAN]; // outer half-width per row
    uint8_t srow[SPRITE_SPAN];
} sprite_t;

typedef struct {
    int16_t sx, sy;
    uint8_t r;
    uint8_t vel;
    uint8_t dshade;
} projected_t;

static uint16_t s_palette[PALETTE_VEL_LEVELS * PALETTE_SHADE_LEVELS];
static sprite_t s_sprites[SPRITE_RADIUS_MAX + 1];

static fluid_point_t *s_points;
static projected_t *s_proj;
static uint16_t *s_order;
static uint16_t *s_bucket;
static uint16_t *s_entry_p;
static int16_t *s_entry_next;
static int16_t s_head[TILE_COUNT];
static uint16_t *s_work;
static int s_count;

static inline uint16_t rgb565(int r, int g, int b)
{
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static void build_palette(void)
{
    // Cold, slow fluid is deep blue; as a particle speeds up it runs through
    // cyan into amber and finally hot orange-red.
    static const float stop_t[4] = {0.0f, 0.35f, 0.70f, 1.0f};
    static const float stop_c[4][3] = {
        {25, 70, 235},
        {0, 190, 255},
        {255, 185, 60},
        {255, 55, 25},
    };

    for (int v = 0; v < PALETTE_VEL_LEVELS; v++) {
        const float t = (float)v / (PALETTE_VEL_LEVELS - 1);
        int seg = 0;
        while (seg < 2 && t > stop_t[seg + 1]) {
            seg++;
        }
        const float span = stop_t[seg + 1] - stop_t[seg];
        const float f = span > 0.0f ? (t - stop_t[seg]) / span : 0.0f;
        const float base[3] = {
            stop_c[seg][0] + (stop_c[seg + 1][0] - stop_c[seg][0]) * f,
            stop_c[seg][1] + (stop_c[seg + 1][1] - stop_c[seg][1]) * f,
            stop_c[seg][2] + (stop_c[seg + 1][2] - stop_c[seg][2]) * f,
        };

        for (int s = 0; s < PALETTE_SHADE_LEVELS; s++) {
            const float k = 0.28f + 0.072f * (float)s;
            s_palette[v * PALETTE_SHADE_LEVELS + s] =
                rgb565((int)(base[0] * k), (int)(base[1] * k), (int)(base[2] * k));
        }
    }
}

static void build_sprites(void)
{
    for (int r = SPRITE_MIN; r <= SPRITE_RADIUS_MAX; r++) {
        sprite_t *sp = &s_sprites[r];
        for (int dy = -r; dy <= r; dy++) {
            const int idx = dy + r;
            const float half = sqrtf((float)(r * r - dy * dy));
            int xo = (int)floorf(half - 0.85f);
            int xe = (int)floorf(half + 0.15f);
            if (xe > r) {
                xe = r;
            }
            if (xo > xe) {
                xo = xe;
            }
            sp->xo[idx] = (int8_t)xo;
            sp->xe[idx] = (int8_t)xe;

            // Light from above: the top of each droplet is a highlight, the
            // bottom falls into shadow. Shading per row (rather than per pixel)
            // keeps the span fill to a single colour.
            const float t = (float)dy / (float)r;
            float b = (0.95f - 0.45f * t) / 1.40f;
            if (b < 0.0f) b = 0.0f;
            if (b > 1.0f) b = 1.0f;
            sp->srow[idx] = (uint8_t)(b * (PALETTE_SHADE_LEVELS - 1) + 0.5f);
        }
    }
}

static void *alloc_int(size_t bytes)
{
    void *p = hal_alloc_fast(bytes);
    if (p == NULL) {
        hal_log(TAG, "out of internal RAM (%u bytes)", (unsigned)bytes);
    }
    return p;
}

hal_err_t render_init(void)
{
    s_count = fluid_count();
    build_palette();
    build_sprites();

    s_points = hal_alloc_bulk(s_count * sizeof(fluid_point_t));
    if (s_points == NULL) {
        s_points = alloc_int(s_count * sizeof(fluid_point_t));
    }
    s_proj = alloc_int(s_count * sizeof(projected_t));
    s_order = alloc_int(s_count * sizeof(uint16_t));
    s_bucket = alloc_int(s_count * sizeof(uint16_t));
    s_entry_p = alloc_int(2 * s_count * sizeof(uint16_t));
    s_entry_next = alloc_int(2 * s_count * sizeof(int16_t));
    s_work = alloc_int(LCD_H_RES * TILE_H * sizeof(uint16_t));

    if (!s_points || !s_proj || !s_order || !s_bucket || !s_entry_p || !s_entry_next || !s_work) {
        return HAL_FAIL;
    }
    s_swap = display_needs_byte_swap();
    hal_log(TAG, "rasteriser ready, %d bands of %dx%d, byte swap %s",
            TILE_COUNT, LCD_H_RES, TILE_H, s_swap ? "on" : "off");
    return HAL_OK;
}

// Perspective projection: particles near the glass are bigger and brighter than
// particles pressed against the back of the case.
static void project(void)
{
    const float cx = LCD_H_RES * 0.5f;
    const float cy = LCD_V_RES * 0.5f;

    for (int i = 0; i < s_count; i++) {
        const fluid_point_t *p = &s_points[i];
        const float scale = PERSPECTIVE_F / (PERSPECTIVE_F + p->z);

        int r = (int)(PARTICLE_RADIUS_PX * scale + 0.5f);
        if (r < SPRITE_MIN) r = SPRITE_MIN;
        if (r > SPRITE_RADIUS_MAX) r = SPRITE_RADIUS_MAX;

        const float depth01 = p->z * (1.0f / (float)BOX_D_PX);
        float dk = 1.0f - DEPTH_DARKEN * (depth01 > 1.0f ? 1.0f : depth01);
        if (dk < 0.0f) dk = 0.0f;

        s_proj[i].sx = (int16_t)(cx + (p->x - cx) * scale);
        s_proj[i].sy = (int16_t)(cy + (p->y - cy) * scale);
        s_proj[i].r = (uint8_t)r;
        s_proj[i].vel = (uint8_t)(p->speed01 * (PALETTE_VEL_LEVELS - 1) + 0.5f);
        s_proj[i].dshade = (uint8_t)(dk * (PALETTE_SHADE_LEVELS - 1) + 0.5f);

        int b = (int)(depth01 * Z_BUCKETS);
        if (b < 0) b = 0;
        if (b >= Z_BUCKETS) b = Z_BUCKETS - 1;
        s_order[i] = (uint16_t)b; // reused below as the bucket key
    }
}

// Counting sort into near-to-far order, then push each particle onto the list
// of every band it touches. Because the lists are built by prepending while
// walking near-to-far, each list ends up ordered far-to-near, which is exactly
// the painter's-algorithm draw order.
static void bin_into_bands(void)
{
    uint16_t counts[Z_BUCKETS + 1];
    memset(counts, 0, sizeof(counts));
    for (int i = 0; i < s_count; i++) {
        counts[s_order[i] + 1]++;
    }
    for (int b = 0; b < Z_BUCKETS; b++) {
        counts[b + 1] += counts[b];
    }
    for (int i = 0; i < s_count; i++) {
        s_bucket[counts[s_order[i]]++] = (uint16_t)i;
    }

    for (int t = 0; t < TILE_COUNT; t++) {
        s_head[t] = -1;
    }

    int e = 0;
    const int pool = 2 * s_count;
    for (int k = 0; k < s_count; k++) {
        const int i = s_bucket[k];
        const int r = s_proj[i].r;
        const int top = s_proj[i].sy - r;
        const int bot = s_proj[i].sy + r;
        if (bot < 0 || top >= LCD_V_RES) {
            continue;
        }
        int t0 = top / TILE_H;
        int t1 = bot / TILE_H;
        if (t0 < 0) t0 = 0;
        if (t1 >= TILE_COUNT) t1 = TILE_COUNT - 1;
        for (int t = t0; t <= t1 && e < pool; t++) {
            s_entry_p[e] = (uint16_t)i;
            s_entry_next[e] = s_head[t];
            s_head[t] = (int16_t)e;
            e++;
        }
    }
}

static inline uint16_t blend_half(uint16_t a, uint16_t b)
{
    return (uint16_t)(((a & 0xF7DE) >> 1) + ((b & 0xF7DE) >> 1));
}

static void draw_particle(uint16_t *band, int y0, const projected_t *p)
{
    const int r = p->r;
    const sprite_t *sp = &s_sprites[r];
    const uint16_t *pal = &s_palette[p->vel * PALETTE_SHADE_LEVELS];
    const int dshade = p->dshade;

    int dy_lo = y0 - p->sy;
    int dy_hi = y0 + TILE_H - 1 - p->sy;
    if (dy_lo < -r) dy_lo = -r;
    if (dy_hi > r) dy_hi = r;

    for (int dy = dy_lo; dy <= dy_hi; dy++) {
        const int idx = dy + r;
        const int shade = (sp->srow[idx] * dshade) / (PALETTE_SHADE_LEVELS - 1);
        const uint16_t colour = pal[shade];
        uint16_t *row = band + (p->sy + dy - y0) * LCD_H_RES;

        const int xo = sp->xo[idx];
        const int xe = sp->xe[idx];

        // Opaque core.
        int a = p->sx - xo;
        int b = p->sx + xo;
        if (a < 0) a = 0;
        if (b > LCD_H_RES - 1) b = LCD_H_RES - 1;
        for (int x = a; x <= b; x++) {
            row[x] = colour;
        }

        // One or two pixels of half-covered edge on each side soften the
        // silhouette; without it particles look like Lego bricks.
        for (int d = xo + 1; d <= xe; d++) {
            const int l = p->sx - d;
            const int rr = p->sx + d;
            if (l >= 0) {
                row[l] = blend_half(row[l], colour);
            }
            if (rr < LCD_H_RES) {
                row[rr] = blend_half(row[rr], colour);
            }
        }
    }
}

// The panel wants big-endian RGB565 but the CPU is little-endian, so the last
// step of every band is a byte swap. Doing it two pixels at a time on 32-bit
// words costs about a millisecond for the whole frame.
static void swap_into(uint32_t *dst, const uint32_t *src, int words)
{
    for (int i = 0; i < words; i++) {
        const uint32_t w = src[i];
        dst[i] = ((w & 0x00FF00FFu) << 8) | ((w >> 8) & 0x00FF00FFu);
    }
}

void render_frame(void)
{
    fluid_snapshot(s_points);
    project();
    bin_into_bands();

    const int band_pixels = LCD_H_RES * TILE_H;
    for (int t = 0; t < TILE_COUNT; t++) {
        memset(s_work, 0, band_pixels * sizeof(uint16_t));

        const int y0 = t * TILE_H;
        for (int e = s_head[t]; e >= 0; e = s_entry_next[e]) {
            draw_particle(s_work, y0, &s_proj[s_entry_p[e]]);
        }

        uint16_t *out = display_acquire_band();
        if (s_swap) {
            swap_into((uint32_t *)out, (const uint32_t *)s_work, band_pixels / 2);
        } else {
            memcpy(out, s_work, band_pixels * sizeof(uint16_t));
        }
        display_send_band(out, t);
    }
}
