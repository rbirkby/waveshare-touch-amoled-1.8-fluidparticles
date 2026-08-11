// Hardware discovery firmware for Waveshare ESP32-S3-Touch-AMOLED-1.8.
//
// Answers the questions the fluid app depends on:
//   1. Which I2C devices are present (and therefore board revision V1 vs V2).
//   2. QMI8658 IMU axis orientation and scaling (hold the board still, read gravity).
//   3. Whether the side PWR button raises an AXP2101 POWERON-short-press interrupt.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_14
#define I2C_HZ 400000

#define ADDR_AXP2101 0x34
#define ADDR_QMI8658 0x6B
#define ADDR_QMI8658_ALT 0x6A

// QMI8658 registers
#define QMI_WHOAMI 0x00
#define QMI_REVISION 0x01
#define QMI_CTRL1 0x02
#define QMI_CTRL2 0x03
#define QMI_CTRL3 0x04
#define QMI_CTRL7 0x08
#define QMI_AX_L 0x35
#define QMI_RESET 0x60

// AXP2101 registers
#define AXP_CHIP_ID 0x03
#define AXP_INTEN1 0x40
#define AXP_INTSTS1 0x48

static const char *TAG = "hwprobe";

static i2c_master_bus_handle_t s_bus;

static esp_err_t dev_add(uint8_t addr, i2c_master_dev_handle_t *out)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, out);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, dst, len, 100);
}

static void scan_bus(void)
{
    static const struct {
        uint8_t addr;
        const char *name;
    } known[] = {
        {0x15, "CST816/CST820 touch (V2 board)"},
        {0x18, "ES8311 audio codec"},
        {0x20, "TCA9554 IO expander"},
        {0x34, "AXP2101 PMIC"},
        {0x38, "FT3168 touch (V1 board)"},
        {0x51, "PCF85063 RTC"},
        {0x6A, "QMI8658 IMU (alt addr)"},
        {0x6B, "QMI8658 IMU"},
    };

    ESP_LOGI(TAG, "--- I2C scan (SDA=%d SCL=%d) ---", I2C_SDA, I2C_SCL);
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) != ESP_OK) {
            continue;
        }
        const char *name = "unknown";
        for (size_t i = 0; i < sizeof(known) / sizeof(known[0]); i++) {
            if (known[i].addr == addr) {
                name = known[i].name;
                break;
            }
        }
        ESP_LOGI(TAG, "  0x%02X  %s", addr, name);
    }
}

static i2c_master_dev_handle_t imu_init(float *accel_scale, float *gyro_scale)
{
    uint8_t addr = ADDR_QMI8658;
    if (i2c_master_probe(s_bus, addr, 50) != ESP_OK) {
        addr = ADDR_QMI8658_ALT;
        if (i2c_master_probe(s_bus, addr, 50) != ESP_OK) {
            ESP_LOGE(TAG, "QMI8658 not found");
            return NULL;
        }
    }

    i2c_master_dev_handle_t dev = NULL;
    ESP_ERROR_CHECK(dev_add(addr, &dev));

    uint8_t id[2] = {0};
    ESP_ERROR_CHECK(reg_read(dev, QMI_WHOAMI, id, sizeof(id)));
    ESP_LOGI(TAG, "QMI8658 @0x%02X WHO_AM_I=0x%02X (expect 0x05) revision=0x%02X", addr, id[0], id[1]);

    ESP_ERROR_CHECK(reg_write(dev, QMI_RESET, 0xB0));
    vTaskDelay(pdMS_TO_TICKS(20));

    // CTRL1: bit6 = address auto-increment for burst reads, little-endian data.
    ESP_ERROR_CHECK(reg_write(dev, QMI_CTRL1, 0x40));
    // CTRL2: accel range 4g (1<<4), ODR 250 Hz (code 5).
    ESP_ERROR_CHECK(reg_write(dev, QMI_CTRL2, (1 << 4) | 5));
    // CTRL3: gyro range 512 dps (5<<4), ODR 224.2 Hz (code 5).
    ESP_ERROR_CHECK(reg_write(dev, QMI_CTRL3, (5 << 4) | 5));
    // CTRL7: enable accelerometer + gyroscope.
    ESP_ERROR_CHECK(reg_write(dev, QMI_CTRL7, 0x03));
    vTaskDelay(pdMS_TO_TICKS(20));

    *accel_scale = 4.0f * 9.80665f / 32768.0f; // m/s^2 per LSB
    *gyro_scale = 512.0f / 32768.0f;           // deg/s per LSB
    return dev;
}

static i2c_master_dev_handle_t pmu_init(void)
{
    if (i2c_master_probe(s_bus, ADDR_AXP2101, 50) != ESP_OK) {
        ESP_LOGE(TAG, "AXP2101 not found");
        return NULL;
    }

    i2c_master_dev_handle_t dev = NULL;
    ESP_ERROR_CHECK(dev_add(ADDR_AXP2101, &dev));

    uint8_t id = 0;
    ESP_ERROR_CHECK(reg_read(dev, AXP_CHIP_ID, &id, 1));
    ESP_LOGI(TAG, "AXP2101 chip id = 0x%02X", id);

    // Clear any latched interrupt status (write-1-to-clear).
    const uint8_t clear[3] = {0xFF, 0xFF, 0xFF};
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(reg_write(dev, AXP_INTSTS1 + i, clear[i]));
    }
    // INTEN2 (0x41): bit0 PWRKEY positive edge, bit1 negative edge,
    // bit2 long press, bit3 short press.
    ESP_ERROR_CHECK(reg_write(dev, AXP_INTEN1 + 1, 0x0F));
    ESP_LOGI(TAG, "AXP2101 PWRKEY interrupts enabled, press the side PWR button now");
    return dev;
}

void app_main(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    scan_bus();

    float accel_scale = 0.0f;
    float gyro_scale = 0.0f;
    i2c_master_dev_handle_t imu = imu_init(&accel_scale, &gyro_scale);
    i2c_master_dev_handle_t pmu = pmu_init();

    int tick = 0;
    while (true) {
        if (imu != NULL && (tick % 5) == 0) {
            uint8_t raw[12] = {0};
            if (reg_read(imu, QMI_AX_L, raw, sizeof(raw)) == ESP_OK) {
                int16_t v[6];
                for (int i = 0; i < 6; i++) {
                    v[i] = (int16_t)((uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8));
                }
                ESP_LOGI(TAG,
                         "accel[m/s^2] x=%7.3f y=%7.3f z=%7.3f  gyro[dps] x=%8.2f y=%8.2f z=%8.2f",
                         v[0] * accel_scale, v[1] * accel_scale, v[2] * accel_scale,
                         v[3] * gyro_scale, v[4] * gyro_scale, v[5] * gyro_scale);
            }
        }

        if (pmu != NULL) {
            uint8_t sts[3] = {0};
            if (reg_read(pmu, AXP_INTSTS1, sts, sizeof(sts)) == ESP_OK &&
                (sts[0] || sts[1] || sts[2])) {
                ESP_LOGW(TAG, "AXP2101 IRQ: STS1=0x%02X STS2=0x%02X STS3=0x%02X%s%s",
                         sts[0], sts[1], sts[2],
                         (sts[1] & 0x08) ? "  <-- PWRKEY SHORT PRESS" : "",
                         (sts[1] & 0x04) ? "  <-- PWRKEY LONG PRESS" : "");
                for (int i = 0; i < 3; i++) {
                    reg_write(pmu, AXP_INTSTS1 + i, sts[i]);
                }
            }
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
