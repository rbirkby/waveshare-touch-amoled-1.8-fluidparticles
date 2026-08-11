// SH8601 AMOLED driver for the RP2350, over a PIO-synthesised QSPI bus.
//
// The panel is the same 368x448 as the ESP32-S3 board's, and the renderer above
// is identical, so this file only has to honour the same four-call contract:
// hand out a free band buffer, take it back, transmit it by DMA, and be able to
// wait for quiet. Two band buffers ping-pong, so the rasteriser fills one while
// the other is on the wire.
//
// Neither board has room for a 322 KB framebuffer -- and this one has no PSRAM
// at all -- which is exactly why the renderer works in horizontal strips.

#include <string.h>

#include "board_config.h"
#include "config.h"
#include "drivers.h"
#include "hal.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "qspi.pio.h"

static const char *TAG = "display";

#define BAND_PIXELS (LCD_H_RES * TILE_H)
#define BAND_BYTES (BAND_PIXELS * 2)
#define NUM_BANDS 2

static PIO s_pio = pio0;
static uint s_sm;
static uint s_off;
static int s_dma;

static uint16_t *s_band[NUM_BANDS];
static int s_next;
static bool s_busy;

// ---------------------------------------------------------------------------
// Low-level bus
// ---------------------------------------------------------------------------
static inline void cs_low(void) { gpio_put(LCD_PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(LCD_PIN_CS, 1); }

// One byte straight out of the four-bit state machine. The pull threshold is 8,
// so the byte has to sit in the top of the word for an MSB-first shift.
static inline void put_byte(uint8_t b)
{
    pio_sm_put_blocking(s_pio, s_sm, (uint32_t)b << 24);
}

// Sends one byte as if the bus were one bit wide.
//
// The state machine always shifts four bits at a time, so a single-wire byte is
// faked by expanding it into four bytes in which only bit 0 of each nibble is
// ever set. Each output nibble therefore drives D0 with one bit of the original
// and holds D1..D3 low, which is exactly what a 1-wire transfer looks like.
// High nibble goes out first, so the bits come out MSB first as SPI requires.
static void write_1wire_byte(uint8_t val)
{
    for (int i = 3; i >= 0; i--) {
        const uint8_t lo = (val & (1u << (2 * i))) ? 1u : 0u;
        const uint8_t hi = (val & (1u << (2 * i + 1))) ? 1u : 0u;
        put_byte((uint8_t)(lo | (hi << 4)));
    }
}

// Waits for the state machine to drain its FIFO and output shift register, so
// every bit is really on the wire before chip select rises.
static void sm_drain(void)
{
    while (!pio_sm_is_tx_fifo_empty(s_pio, s_sm)) {
        tight_loop_contents();
    }
    // The last byte is still in the shifter; two cycles per nibble at sys_clk/2.
    busy_wait_us(2);
}

// The panel frames every register access as command 0x02 followed by a 24-bit
// address whose middle byte is the register number, all in single-wire mode.
static void write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    cs_low();
    write_1wire_byte(0x02);
    write_1wire_byte(0x00);
    write_1wire_byte(reg);
    write_1wire_byte(0x00);
    for (size_t i = 0; i < len; i++) {
        write_1wire_byte(data[i]);
    }
    sm_drain();
    cs_high();
}

static void write_cmd(uint8_t reg)
{
    write_reg(reg, NULL, 0);
}

// Opens a pixel stream and leaves chip select asserted. Command 0x32 is "write
// continue on four wires": the header goes out single-wire, then the pixel bytes
// that follow are taken four bits at a time.
static void begin_pixels(void)
{
    cs_low();
    write_1wire_byte(0x32);
    write_1wire_byte(0x00);
    write_1wire_byte(0x2C);
    write_1wire_byte(0x00);
    sm_drain();
}

static void set_window(int x0, int y0, int x1, int y1)
{
    const uint8_t col[4] = {(uint8_t)((x0 + LCD_X_GAP) >> 8), (uint8_t)(x0 + LCD_X_GAP),
                            (uint8_t)((x1 + LCD_X_GAP) >> 8), (uint8_t)(x1 + LCD_X_GAP)};
    const uint8_t row[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0,
                            (uint8_t)(y1 >> 8), (uint8_t)y1};
    write_reg(0x2A, col, 4);
    write_reg(0x2B, row, 4);
}

static void start_dma(const void *src, size_t bytes)
{
    dma_channel_config c = dma_channel_get_default_config(s_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(s_pio, s_sm, true));
    dma_channel_configure(s_dma, &c, &s_pio->txf[s_sm], src, bytes, true);
}

// ---------------------------------------------------------------------------
// Panel bring-up
// ---------------------------------------------------------------------------
static void panel_reset(void)
{
    gpio_init(LCD_PIN_RST);
    gpio_set_dir(LCD_PIN_RST, GPIO_OUT);
    gpio_put(LCD_PIN_RST, 1);
    sleep_ms(50);
    gpio_put(LCD_PIN_RST, 0);
    sleep_ms(50);
    gpio_put(LCD_PIN_RST, 1);
    sleep_ms(300);
}

// Register order and values are taken verbatim from Waveshare's driver for this
// exact panel (C/01-LCD/lib/AMOLED/AMOLED_1in8.c in their demo pack).
//
// Do not borrow this sequence from their 1.75" board: the panels differ, and in
// particular the 1.75" wants 0x3A = 0x05 for 16-bit colour where this one wants
// 0x55. That single byte is the difference between a picture and a blank screen.
static void panel_init_regs(void)
{
    write_cmd(0x11); // sleep out
    sleep_ms(120);

    const uint8_t tear[2] = {0x01, 0xC5};
    write_reg(0x44, tear, 2); // tear scanline

    const uint8_t te = 0x00;
    write_reg(0x35, &te, 1); // tearing effect line on

    const uint8_t colmod = 0x55;
    write_reg(0x3A, &colmod, 1); // 16 bits per pixel, RGB565

    const uint8_t iface = 0x80;
    write_reg(0xC4, &iface, 1); // QSPI interface config

    const uint8_t ctrl = 0x20;
    write_reg(0x53, &ctrl, 1); // brightness control on

    const uint8_t bright = 0xFF;
    write_reg(0x51, &bright, 1); // full brightness

    write_cmd(0x29); // display on
    sleep_ms(10);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------
hal_err_t display_init(void)
{
    gpio_init(LCD_PIN_CS);
    gpio_set_dir(LCD_PIN_CS, GPIO_OUT);
    cs_high();

    panel_reset();

    s_off = pio_add_program(s_pio, &qspi_tx_program);
    s_sm = pio_claim_unused_sm(s_pio, true);
    qspi_tx_program_init(s_pio, s_sm, s_off, LCD_PIN_PCLK, LCD_PIN_D0, LCD_PIO_CLKDIV);

    s_dma = dma_claim_unused_channel(true);

    panel_init_regs();

    for (int i = 0; i < NUM_BANDS; i++) {
        s_band[i] = hal_alloc_dma(BAND_BYTES);
        if (s_band[i] == NULL) {
            hal_log(TAG, "out of RAM for band buffers");
            return HAL_FAIL;
        }
    }

    // Paint the panel black once so nothing stale shows through.
    memset(s_band[0], 0, BAND_BYTES);
    for (int b = 0; b < TILE_COUNT; b++) {
        set_window(0, b * TILE_H, LCD_H_RES - 1, b * TILE_H + TILE_H - 1);
        begin_pixels();
        start_dma(s_band[0], BAND_BYTES);
        dma_channel_wait_for_finish_blocking(s_dma);
        sm_drain();
        cs_high();
    }

    hal_log(TAG, "panel %dx%d, PIO QSPI at %u MHz, %d bands of %dx%d",
            LCD_H_RES, LCD_V_RES,
            (unsigned)(clock_get_hz(clk_sys) / (2u * 1000000u * (unsigned)LCD_PIO_CLKDIV)),
            TILE_COUNT, LCD_H_RES, TILE_H);
    return HAL_OK;
}

// Ends whatever transfer is in flight and leaves the bus quiet and deselected.
// The DMA completing only means the bytes reached the FIFO, so the state
// machine has to be drained too before chip select may rise, otherwise the tail
// of the band is cut off.
static void finish_transfer(void)
{
    if (!s_busy) {
        return;
    }
    dma_channel_wait_for_finish_blocking(s_dma);
    sm_drain();
    cs_high();
    s_busy = false;
}

uint16_t *display_acquire_band(void)
{
    // Only one transfer can be in flight, so the buffer we are about to reuse is
    // free once that transfer has drained.
    finish_transfer();
    uint16_t *buf = s_band[s_next];
    s_next = (s_next + 1) % NUM_BANDS;
    return buf;
}

void display_send_band(uint16_t *buf, int band)
{
    // The window and pixel-stream headers must not be injected into the middle
    // of the previous band's pixel stream.
    finish_transfer();

    // Each band is its own little window, so a slow frame can never smear a
    // strip into the wrong place.
    set_window(0, band * TILE_H, LCD_H_RES - 1, band * TILE_H + TILE_H - 1);
    begin_pixels();
    start_dma(buf, BAND_BYTES);
    s_busy = true;
}

void display_wait_idle(void)
{
    finish_transfer();
}

// The panel wants the high byte of each RGB565 pixel first, and the DMA streams
// bytes out of the band buffer in memory order, so the renderer has to store
// them big-endian. (Waveshare's own LVGL port sets LV_COLOR_16_SWAP for the
// same reason.)
bool display_needs_byte_swap(void)
{
    return true;
}
