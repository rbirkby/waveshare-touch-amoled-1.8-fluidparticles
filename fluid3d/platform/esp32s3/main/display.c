// CO5300 AMOLED driver glue.
//
// The panel is 368x448 RGB565 behind a QSPI bus running at 40 MHz, which is
// 20 MB/s, or about 16.5 ms for a whole 330 KB frame. That is far too big to
// keep two copies of in the 512 KB of internal SRAM, so instead the frame is
// pushed out as 14 horizontal bands of 32 rows. Two band buffers are cycled:
// the CPU draws into one while DMA streams the other to the panel.

#include "drivers.h"

#include "board.h"
#include "config.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BAND_PIXELS (LCD_H_RES * TILE_H)
#define BAND_BYTES (BAND_PIXELS * 2)
#define BAND_BUFFERS 2

static const char *TAG = "display";

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_bands[BAND_BUFFERS];
static int s_next;
static SemaphoreHandle_t s_free_slots;

// Init sequence lifted from the Waveshare BSP / Espressif CO5300 example.
static const co5300_lcd_init_cmd_t s_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},                   // command page 0
    {0xC4, (uint8_t[]){0x80}, 1, 0},                   // QSPI write mode
    {0x3A, (uint8_t[]){0x55}, 1, 0},                   // 16 bits per pixel
    {0x35, (uint8_t[]){0x00}, 1, 0},                   // tearing effect on
    {0x53, (uint8_t[]){0x20}, 1, 0},                   // brightness control on
    {0x51, (uint8_t[]){0xFF}, 1, 0},                   // full brightness
    {0x63, (uint8_t[]){0xFF}, 1, 0},                   // HBM brightness
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0}, // columns 0..367
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0}, // rows 0..447
    {0x11, NULL, 0, 100},                              // sleep out
    {0x29, NULL, 0, 0},                                // display on
};

static bool IRAM_ATTR on_trans_done(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *data,
                                    void *ctx)
{
    (void)io;
    (void)data;
    (void)ctx;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_free_slots, &woken);
    return woken == pdTRUE;
}

hal_err_t display_init(void)
{
    s_free_slots = xSemaphoreCreateCounting(BAND_BUFFERS, BAND_BUFFERS);
    ESP_RETURN_ON_FALSE(s_free_slots, ESP_ERR_NO_MEM, TAG, "semaphore");

    for (int i = 0; i < BAND_BUFFERS; i++) {
        s_bands[i] = heap_caps_aligned_alloc(64, BAND_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_RETURN_ON_FALSE(s_bands[i], ESP_ERR_NO_MEM, TAG, "band buffer %d", i);
    }

    const spi_bus_config_t bus_cfg = CO5300_PANEL_BUS_QSPI_CONFIG(
        LCD_PIN_PCLK, LCD_PIN_D0, LCD_PIN_D1, LCD_PIN_D2, LCD_PIN_D3, BAND_BYTES);
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "spi bus");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg =
        CO5300_PANEL_IO_QSPI_CONFIG(LCD_PIN_CS, on_trans_done, NULL);
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &io), TAG, "panel io");

    const co5300_vendor_config_t vendor_cfg = {
        .init_cmds = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .flags.use_qspi_interface = 1,
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = (void *)&vendor_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(io, &panel_cfg, &s_panel), TAG, "panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");

    // The V2 revision of this board shifts the visible window by 16 columns.
    // Waveshare's own examples detect it by probing for the CST816 touch chip.
    i2c_master_bus_handle_t bus = board_i2c_bus();
    const bool is_v2 = bus != NULL && i2c_master_probe(bus, ADDR_TOUCH_CST816, 50) == ESP_OK;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, is_v2 ? 0x10 : 0, 0), TAG, "gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "display on");

    ESP_LOGI(TAG, "CO5300 %dx%d up, %s panel offset, %d KB of band buffers",
             LCD_H_RES, LCD_V_RES, is_v2 ? "V2" : "V1", (BAND_BYTES * BAND_BUFFERS) / 1024);
    return ESP_OK;
}

uint16_t *display_acquire_band(void)
{
    xSemaphoreTake(s_free_slots, portMAX_DELAY);
    uint16_t *buf = s_bands[s_next];
    s_next = (s_next + 1) % BAND_BUFFERS;
    return buf;
}

void display_send_band(uint16_t *buf, int band)
{
    const int y0 = band * TILE_H;
    esp_lcd_panel_draw_bitmap(s_panel, 0, y0, LCD_H_RES, y0 + TILE_H, buf);
}

void display_wait_idle(void)
{
    for (int i = 0; i < BAND_BUFFERS; i++) {
        xSemaphoreTake(s_free_slots, portMAX_DELAY);
    }
    for (int i = 0; i < BAND_BUFFERS; i++) {
        xSemaphoreGive(s_free_slots);
    }
}

// The ESP32-S3 pushes RGB565 out of a byte-oriented SPI peripheral, so the two
// halves of each pixel arrive in the wrong order and the renderer has to swap
// them as it copies into the DMA buffer.
bool display_needs_byte_swap(void)
{
    return true;
}
