// QMI8658 6-axis IMU and AXP2101 power button, for the RP2350 board.
//
// Both hang off the single 400 kHz I2C bus on GPIO 6/7. This is deliberately
// the same logic as the ESP32-S3 board's driver, because both boards carry the
// same two parts -- only the bus plumbing underneath differs.
//
// The user button is not wired to a GPIO. It goes to the AXP2101's PWRKEY pin,
// and the PMIC latches an interrupt flag when it is pressed. Polling that flag
// over I2C is cheap and, unlike claiming the PMIC's interrupt line, it cannot
// interfere with the PMIC's own long-press power-off behaviour, so holding the
// button still switches the board off and reboot/reflash keep working.

#include "drivers.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "board_config.h"
#include "config.h"
#include "hal.h"
#include "pico/stdlib.h"

// QMI8658 registers
#define QMI_WHOAMI 0x00
#define QMI_CTRL1 0x02
#define QMI_CTRL2 0x03
#define QMI_CTRL3 0x04
#define QMI_CTRL7 0x08
#define QMI_AX_L 0x35
#define QMI_RESET 0x60

// AXP2101 registers. INTEN2/INTSTS2 hold the four PWRKEY edge flags in bits
// 0..3: positive edge, negative edge, long press, short press.
#define AXP_CHIP_ID 0x03
#define AXP_INTEN2 0x41
#define AXP_INTSTS1 0x48
#define AXP_INTSTS2 0x49
#define AXP_PWRKEY_MASK 0x0F
#define AXP_PWRKEY_RELEASE_MASK 0x0A // negative edge | short press

static const char *TAG = "sensors";

static uint8_t s_imu;
static bool s_have_pmu;
static float s_accel_scale;
static float s_gyro_scale;
static float s_gyro_bias[3];

static hal_err_t imu_init(void)
{
    s_imu = 0;
    if (board_i2c_probe(ADDR_QMI8658)) {
        s_imu = ADDR_QMI8658;
    } else if (board_i2c_probe(ADDR_QMI8658_ALT)) {
        s_imu = ADDR_QMI8658_ALT;
    }
    if (s_imu == 0) {
        hal_log(TAG, "QMI8658 not found");
        return HAL_FAIL;
    }

    uint8_t who = 0;
    board_reg_read(s_imu, QMI_WHOAMI, &who, 1);
    if (who != 0x05) {
        hal_log(TAG, "unexpected QMI8658 WHO_AM_I 0x%02X", who);
    }

    board_reg_write(s_imu, QMI_RESET, 0xB0);
    sleep_ms(20);

    board_reg_write(s_imu, QMI_CTRL1, 0x40);         // auto-increment reads
    board_reg_write(s_imu, QMI_CTRL2, (1 << 4) | 5); // accel +-4 g @ 250 Hz
    board_reg_write(s_imu, QMI_CTRL3, (5 << 4) | 5); // gyro +-512 dps @ 224 Hz
    board_reg_write(s_imu, QMI_CTRL7, 0x03);         // enable both
    sleep_ms(30);

    s_accel_scale = 4.0f * 9.80665f / 32768.0f;
    s_gyro_scale = (512.0f / 32768.0f) * (float)M_PI / 180.0f;
    hal_log(TAG, "QMI8658 at 0x%02X", s_imu);
    return HAL_OK;
}

static hal_err_t imu_read_raw(float *a, float *g)
{
    uint8_t raw[12];
    if (!board_reg_read(s_imu, QMI_AX_L, raw, sizeof(raw))) {
        return HAL_FAIL;
    }
    int16_t v[6];
    for (int i = 0; i < 6; i++) {
        v[i] = (int16_t)((uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8));
    }
    for (int i = 0; i < 3; i++) {
        a[i] = v[i] * s_accel_scale;
        g[i] = v[i + 3] * s_gyro_scale;
    }
    return HAL_OK;
}

// The gyro has a fixed offset of several degrees per second. Left uncorrected
// it would make the fluid spin forever, so average half a second of samples at
// boot while the board is assumed to be sitting still.
static void gyro_calibrate(void)
{
    float a[3], g[3];
    double sum[3] = {0, 0, 0};
    int n = 0;
    for (int i = 0; i < 100; i++) {
        if (imu_read_raw(a, g) == HAL_OK) {
            for (int k = 0; k < 3; k++) {
                sum[k] += g[k];
            }
            n++;
        }
        sleep_ms(5);
    }
    if (n > 0) {
        for (int k = 0; k < 3; k++) {
            s_gyro_bias[k] = (float)(sum[k] / n);
        }
    }
    hal_log(TAG, "gyro bias %.3f %.3f %.3f rad/s",
            s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
}

static void pmu_init(void)
{
    if (!board_i2c_probe(ADDR_AXP2101)) {
        hal_log(TAG, "AXP2101 not found, button reset disabled");
        return;
    }
    uint8_t id = 0;
    board_reg_read(ADDR_AXP2101, AXP_CHIP_ID, &id, 1);

    // Interrupt status registers are write-1-to-clear. Clearing everything
    // latched during power-on stops the first poll reading a phantom press.
    for (int i = 0; i < 3; i++) {
        board_reg_write(ADDR_AXP2101, AXP_INTSTS1 + i, 0xFF);
    }
    board_reg_write(ADDR_AXP2101, AXP_INTEN2, AXP_PWRKEY_MASK);
    s_have_pmu = true;
    hal_log(TAG, "AXP2101 0x%02X ready, button resets the simulation", id);
}

hal_err_t sensors_init(void)
{
    if (imu_init() != HAL_OK) {
        return HAL_FAIL;
    }
    gyro_calibrate();
    pmu_init();
    return HAL_OK;
}

hal_err_t sensors_read(sensors_sample_t *out)
{
    float a[3], g[3];
    if (imu_read_raw(a, g) != HAL_OK) {
        return HAL_FAIL;
    }
    for (int k = 0; k < 3; k++) {
        g[k] -= s_gyro_bias[k];
    }

#if IMU_SWAP_XY
    const int ix = 1, iy = 0;
#else
    const int ix = 0, iy = 1;
#endif
    out->ax = IMU_SIGN_X * a[ix];
    out->ay = IMU_SIGN_Y * a[iy];
    out->az = IMU_SIGN_Z * a[2];
    out->gx = IMU_SIGN_X * g[ix];
    out->gy = IMU_SIGN_Y * g[iy];
    out->gz = IMU_SIGN_Z * g[2];
    return HAL_OK;
}

bool button_pressed(void)
{
    if (!s_have_pmu) {
        return false;
    }
    uint8_t sts = 0;
    if (!board_reg_read(ADDR_AXP2101, AXP_INTSTS2, &sts, 1)) {
        return false;
    }
    const uint8_t hit = sts & AXP_PWRKEY_MASK;
    if (hit == 0) {
        return false;
    }
    board_reg_write(ADDR_AXP2101, AXP_INTSTS2, hit); // clear what we consumed
    return (hit & AXP_PWRKEY_RELEASE_MASK) != 0;
}
