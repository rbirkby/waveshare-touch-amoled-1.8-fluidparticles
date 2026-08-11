// QMI8658 6-axis IMU and AXP2101 power button.
//
// Both hang off the single 400 kHz I2C bus on GPIO15/GPIO14.
//
// The side PWR button is not wired to a GPIO. It goes to the AXP2101's PWRKEY
// pin, and the PMIC latches an interrupt flag when it is pressed. Polling that
// flag over I2C is cheap and, unlike claiming the PMIC's interrupt line, it
// cannot interfere with the PMIC's own long-press power-off behaviour, so
// holding the button still switches the board off and reboot/reflash keep
// working exactly as they did.

#include "drivers.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

static i2c_master_dev_handle_t s_imu;
static i2c_master_dev_handle_t s_pmu;
static float s_accel_scale;
static float s_gyro_scale;
static float s_gyro_bias[3];

static esp_err_t imu_init(void)
{
    s_imu = board_i2c_add(ADDR_QMI8658);
    if (s_imu == NULL) {
        s_imu = board_i2c_add(ADDR_QMI8658_ALT);
    }
    if (s_imu == NULL) {
        ESP_LOGE(TAG, "QMI8658 not found");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t who = 0;
    board_reg_read(s_imu, QMI_WHOAMI, &who, 1);
    if (who != 0x05) {
        ESP_LOGW(TAG, "unexpected QMI8658 WHO_AM_I 0x%02X", who);
    }

    board_reg_write(s_imu, QMI_RESET, 0xB0);
    vTaskDelay(pdMS_TO_TICKS(20));

    board_reg_write(s_imu, QMI_CTRL1, 0x40);            // auto-increment reads
    board_reg_write(s_imu, QMI_CTRL2, (1 << 4) | 5);    // accel +-4 g @ 250 Hz
    board_reg_write(s_imu, QMI_CTRL3, (5 << 4) | 5);    // gyro +-512 dps @ 224 Hz
    board_reg_write(s_imu, QMI_CTRL7, 0x03);            // enable both
    vTaskDelay(pdMS_TO_TICKS(30));

    s_accel_scale = 4.0f * 9.80665f / 32768.0f;
    s_gyro_scale = (512.0f / 32768.0f) * (float)M_PI / 180.0f;
    return ESP_OK;
}

static esp_err_t imu_read_raw(float *a, float *g)
{
    uint8_t raw[12];
    esp_err_t err = board_reg_read(s_imu, QMI_AX_L, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    int16_t v[6];
    for (int i = 0; i < 6; i++) {
        v[i] = (int16_t)((uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8));
    }
    for (int i = 0; i < 3; i++) {
        a[i] = v[i] * s_accel_scale;
        g[i] = v[i + 3] * s_gyro_scale;
    }
    return ESP_OK;
}

// The gyro has a fixed offset of several degrees per second. Left uncorrected
// it would make the fluid spin forever, so average a second of samples at boot
// while the board is assumed to be sitting still.
static void gyro_calibrate(void)
{
    float a[3], g[3];
    double sum[3] = {0, 0, 0};
    int n = 0;
    for (int i = 0; i < 100; i++) {
        if (imu_read_raw(a, g) == ESP_OK) {
            for (int k = 0; k < 3; k++) {
                sum[k] += g[k];
            }
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n > 0) {
        for (int k = 0; k < 3; k++) {
            s_gyro_bias[k] = (float)(sum[k] / n);
        }
    }
    ESP_LOGI(TAG, "gyro bias %.3f %.3f %.3f rad/s", s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
}

static esp_err_t pmu_init(void)
{
    s_pmu = board_i2c_add(ADDR_AXP2101);
    if (s_pmu == NULL) {
        ESP_LOGW(TAG, "AXP2101 not found, PWR button reset disabled");
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t id = 0;
    board_reg_read(s_pmu, AXP_CHIP_ID, &id, 1);

    // Interrupt status registers are write-1-to-clear.
    for (int i = 0; i < 3; i++) {
        board_reg_write(s_pmu, AXP_INTSTS1 + i, 0xFF);
    }
    board_reg_write(s_pmu, AXP_INTEN2, AXP_PWRKEY_MASK);
    ESP_LOGI(TAG, "AXP2101 0x%02X ready, PWR button resets the simulation", id);
    return ESP_OK;
}

hal_err_t sensors_init(void)
{
    esp_err_t err = imu_init();
    if (err != ESP_OK) {
        return err;
    }
    gyro_calibrate();
    pmu_init();
    return ESP_OK;
}

hal_err_t sensors_read(sensors_sample_t *out)
{
    float a[3], g[3];
    esp_err_t err = imu_read_raw(a, g);
    if (err != ESP_OK) {
        return err;
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
    return ESP_OK;
}

bool button_pressed(void)
{
    if (s_pmu == NULL) {
        return false;
    }
    uint8_t sts = 0;
    if (board_reg_read(s_pmu, AXP_INTSTS2, &sts, 1) != ESP_OK) {
        return false;
    }
    const uint8_t hit = sts & AXP_PWRKEY_MASK;
    if (hit == 0) {
        return false;
    }
    board_reg_write(s_pmu, AXP_INTSTS2, hit); // clear the flags we consumed
    return (hit & AXP_PWRKEY_RELEASE_MASK) != 0;
}
