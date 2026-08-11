# The portable half of the application lives in fluid3d/src and is shared
# verbatim with the RP2350 port. Only the files in this directory are specific
# to the ESP32-S3.
set(CORE ${CMAKE_CURRENT_LIST_DIR}/../../../src)

idf_component_register(
    SRCS
        "main.c"
        "hal_esp32s3.c"
        "board.c"
        "display.c"
        "sensors.c"
        "touch.c"
        "${CORE}/app.c"
        "${CORE}/fluid.c"
        "${CORE}/render.c"
    INCLUDE_DIRS "." "${CORE}"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_spi esp_lcd esp_lcd_co5300 esp_timer)
