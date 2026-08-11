// Proves, without anyone looking at the screen, whether the AMOLED is actually
// being driven.
//
// Two independent checks:
//
//  1. Power. An AMOLED lights each pixel individually, so a full white frame
//     draws far more current than a full black one. The AXP2101 PMIC can
//     measure the supply rails, so alternating white and black and watching for
//     a correlated change is direct evidence that pixels are lighting up.
//
//  2. Tearing effect. Register 0x35 asks the panel to pulse a TE signal once
//     per refresh. It is not in Waveshare's pin list, but if it is routed to
//     any GPIO at all then that pin will toggle at roughly 60 Hz once the panel
//     is running -- which also proves the panel accepted our commands.
//
// The bus setup here is a deliberate copy of platform/rp2350/display.c so that
// what is proven is what actually ships.

#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "qspi.pio.h"

#define PIN_CS 9
#define PIN_CLK 10
#define PIN_D0 11
#define PIN_RST 15
#define PIN_PWR 17
#define I2C_SDA 6
#define I2C_SCL 7
#define AXP 0x34

#define W 368
#define H 448
#define BAND_H 32
#define BAND_PX (W * BAND_H)

static PIO pio = pio0;
static uint sm, off;
static int dma_ch;
static uint16_t band[BAND_PX];

static inline void cs_low(void) { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }
static inline void put_byte(uint8_t b) { pio_sm_put_blocking(pio, sm, (uint32_t)b << 24); }

static void w1(uint8_t val)
{
    for (int i = 3; i >= 0; i--) {
        const uint8_t lo = (val & (1u << (2 * i))) ? 1u : 0u;
        const uint8_t hi = (val & (1u << (2 * i + 1))) ? 1u : 0u;
        put_byte((uint8_t)(lo | (hi << 4)));
    }
}

static void drain(void)
{
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) {
        tight_loop_contents();
    }
    busy_wait_us(20);
}

static void wreg(uint8_t reg, const uint8_t *d, size_t n)
{
    cs_low();
    w1(0x02);
    w1(0x00);
    w1(reg);
    w1(0x00);
    for (size_t i = 0; i < n; i++) {
        w1(d[i]);
    }
    drain();
    cs_high();
}

static void wcmd(uint8_t reg) { wreg(reg, NULL, 0); }

static void set_window(int x0, int y0, int x1, int y1)
{
    const uint8_t c[4] = {(uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1};
    const uint8_t r[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1};
    wreg(0x2A, c, 4);
    wreg(0x2B, r, 4);
}

static void fill(uint16_t colour_be)
{
    for (int i = 0; i < BAND_PX; i++) {
        band[i] = colour_be;
    }
    for (int b = 0; b < H / BAND_H; b++) {
        set_window(0, b * BAND_H, W - 1, b * BAND_H + BAND_H - 1);
        cs_low();
        w1(0x32);
        w1(0x00);
        w1(0x2C);
        w1(0x00);
        drain();
        dma_channel_config c = dma_channel_get_default_config(dma_ch);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
        dma_channel_configure(dma_ch, &c, &pio->txf[sm], band, BAND_PX * 2, true);
        dma_channel_wait_for_finish_blocking(dma_ch);
        drain();
        cs_high();
    }
}

static bool rd(uint8_t reg, uint8_t *dst, size_t n)
{
    if (i2c_write_blocking(i2c1, AXP, &reg, 1, true) < 0) {
        return false;
    }
    return i2c_read_blocking(i2c1, AXP, dst, n, false) >= 0;
}

static void wr(uint8_t reg, uint8_t v)
{
    uint8_t b[2] = {reg, v};
    i2c_write_blocking(i2c1, AXP, b, 2, false);
}

static int adc(uint8_t reg_h)
{
    uint8_t v[2];
    if (!rd(reg_h, v, 2)) {
        return -1;
    }
    return ((int)v[0] << 8) | v[1];
}

int main(void)
{
    set_sys_clock_khz(200000, true);
    stdio_init_all();
    sleep_ms(3000);
    printf("\n=== AMOLED display probe ===\n");

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    wr(0x30, 0x1F); // enable every PMIC ADC channel
    sleep_ms(50);

    gpio_init(PIN_PWR);
    gpio_set_dir(PIN_PWR, GPIO_OUT);
    gpio_put(PIN_PWR, 1);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1);
    sleep_ms(50);
    gpio_put(PIN_RST, 0);
    sleep_ms(50);
    gpio_put(PIN_RST, 1);
    sleep_ms(300);

    off = pio_add_program(pio, &qspi_tx_program);
    sm = pio_claim_unused_sm(pio, true);
    qspi_tx_program_init(pio, sm, off, PIN_CLK, PIN_D0, 2.0f);
    dma_ch = dma_claim_unused_channel(true);

    // Exactly the sequence in Waveshare's AMOLED_1in8.c.
    wcmd(0x11);
    sleep_ms(120);
    const uint8_t tear[2] = {0x01, 0xC5};
    wreg(0x44, tear, 2);
    const uint8_t te = 0x00;
    wreg(0x35, &te, 1);
    const uint8_t cm = 0x55;
    wreg(0x3A, &cm, 1);
    const uint8_t ifc = 0x80;
    wreg(0xC4, &ifc, 1);
    const uint8_t ctl = 0x20;
    wreg(0x53, &ctl, 1);
    const uint8_t br = 0xFF;
    wreg(0x51, &br, 1);
    wcmd(0x29);
    sleep_ms(10);
    printf("panel init sent\n");

    // --- check 2: is any GPIO pulsing? ------------------------------------
    const int cand[] = {0, 1, 2, 3, 8, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};
    printf("scanning for a TE pulse...\n");
    bool found = false;
    for (unsigned k = 0; k < sizeof(cand) / sizeof(cand[0]); k++) {
        const int p = cand[k];
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_disable_pulls(p);
        int edges = 0;
        int prev = gpio_get(p);
        absolute_time_t end = make_timeout_time_ms(200);
        while (!time_reached(end)) {
            const int now = gpio_get(p);
            if (now != prev) {
                edges++;
                prev = now;
            }
        }
        if (edges > 4) {
            printf("  GPIO %-2d toggling: %d edges in 200 ms (~%d Hz) <-- likely TE\n",
                   p, edges, edges / 2 * 5);
            found = true;
        }
    }
    if (!found) {
        printf("  no pulsing GPIO (TE may simply not be routed on this board)\n");
    }

    // --- check 1: does a white frame cost more power than a black one? ----
    printf("\nalternating white and black frames, watching the PMIC:\n");
    long sum_w = 0, sum_b = 0;
    int n = 0;
    for (int round = 0; round < 6; round++) {
        fill(0xFFFF);
        sleep_ms(400);
        const int vw = adc(0x34);
        const int sw = adc(0x3A);

        fill(0x0000);
        sleep_ms(400);
        const int vb = adc(0x34);
        const int sb = adc(0x3A);

        printf("  round %d: white vbat=%d vsys=%d | black vbat=%d vsys=%d | dvbat=%d dvsys=%d\n",
               round, vw, sw, vb, sb, vw - vb, sw - sb);
        sum_w += sw;
        sum_b += sb;
        n++;
    }
    printf("\nmean vsys  white=%ld  black=%ld  difference=%ld\n",
           sum_w / n, sum_b / n, (sum_w - sum_b) / n);
    printf("a consistently LOWER white reading means the panel is lighting pixels\n");

    // Leave something unmistakable behind for a human to glance at.
    fill(0x00F8); // RGB565 red, byte order as this panel wants it
    printf("\nscreen left filled RED. done.\n");

    while (true) {
        tight_loop_contents();
    }
}
