// Button investigation: is the user button GPIO18, another GPIO, or the PMIC?
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define SDA 6
#define SCL 7
#define I2C i2c1
#define AXP 0x34

static bool rd(uint8_t a, uint8_t r, uint8_t *d, int n) {
    if (i2c_write_blocking_until(I2C, a, &r, 1, true, make_timeout_time_ms(10)) != 1) return false;
    return i2c_read_blocking_until(I2C, a, d, n, false, make_timeout_time_ms(10)) == n;
}
static bool wr(uint8_t a, uint8_t r, uint8_t v) {
    uint8_t b[2] = {r, v};
    return i2c_write_blocking_until(I2C, a, b, 2, false, make_timeout_time_ms(10)) == 2;
}

int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    printf("\n=== button investigation ===\n");

    i2c_init(I2C, 400000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA); gpio_pull_up(SCL);

    // Which GPIOs look like an idle active-low button (float high with pull-up,
    // follow a pull-down)? Skip the pins we know are in use.
    const int used[] = {6,7,9,10,11,12,13,14,15,17,4,5,8};
    printf("\n-- GPIO pull test (up/down) --\n");
    for (int p = 0; p <= 29; p++) {
        bool skip = false;
        for (unsigned i = 0; i < sizeof(used)/sizeof(used[0]); i++) if (used[i] == p) skip = true;
        if (skip) continue;
        gpio_init(p); gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);   sleep_ms(2); int hi = gpio_get(p);
        gpio_pull_down(p); sleep_ms(2); int lo = gpio_get(p);
        gpio_disable_pulls(p);
        const char *verdict = (hi && !lo) ? "floating (button candidate)"
                            : (hi && lo)  ? "driven HIGH"
                            : (!hi && !lo)? "driven LOW"
                                          : "odd";
        printf("  gp%-2d up=%d down=%d  %s\n", p, hi, lo, verdict);
    }

    // AXP2101 power key, exactly as on the ESP32-S3 board.
    printf("\n-- AXP2101 PWRKEY --\n");
    uint8_t v;
    if (rd(AXP, 0x41, &v, 1)) printf("  INTEN2 (0x41) = 0x%02X\n", v);
    wr(AXP, 0x41, v | 0x0F);                 // enable power-key IRQs
    if (rd(AXP, 0x41, &v, 1)) printf("  INTEN2 now  = 0x%02X\n", v);
    if (rd(AXP, 0x49, &v, 1)) { printf("  INTSTS2 (0x49) = 0x%02X\n", v); wr(AXP, 0x49, v); }

    printf("\n-- watching 25 s: press the user button if you can --\n");
    uint8_t last = 0;
    for (int i = 0; i < 50; i++) {
        if (rd(AXP, 0x49, &v, 1) && v) {
            printf("  [%2d] AXP2101 INTSTS2 = 0x%02X  <-- PWRKEY EVENT\n", i, v);
            wr(AXP, 0x49, v);
        }
        printf("  [%2d] gp18=%d gp0=%d gp1=%d gp2=%d gp3=%d gp16=%d gp19=%d gp22=%d\n",
               i, gpio_get(18), gpio_get(0), gpio_get(1), gpio_get(2), gpio_get(3),
               gpio_get(16), gpio_get(19), gpio_get(22));
        sleep_ms(500);
    }
    printf("done\n");
    while (1) tight_loop_contents();
}
