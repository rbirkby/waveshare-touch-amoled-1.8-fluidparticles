cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(fluid3d C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

# The portable half of the application. Nothing in here knows what a Pico is.
set(CORE ${CMAKE_CURRENT_LIST_DIR}/../../src)

add_executable(fluid3d
    ${CORE}/app.c
    ${CORE}/fluid.c
    ${CORE}/render.c
    main.c
    hal_rp2350.c
    board.c
    display.c
    sensors.c
    touch.c
)

pico_generate_pio_header(fluid3d ${CMAKE_CURRENT_LIST_DIR}/qspi.pio)

target_include_directories(fluid3d PRIVATE ${CORE} ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(fluid3d
    pico_stdlib
    pico_multicore
    pico_rand
    hardware_i2c
    hardware_pio
    hardware_dma
    hardware_clocks
)

# -O2 rather than -Os: the solver is the bottleneck and it inlines heavily.
# The M33 has hardware single-precision divide and square root, so keep the
# float maths single precision and let the FPU do it rather than calling libm.
target_compile_options(fluid3d PRIVATE
    -O2
    -ffast-math
    -fsingle-precision-constant
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# USB serial for the stats line; the UART pins are not broken out usefully.
pico_enable_stdio_usb(fluid3d 1)
pico_enable_stdio_uart(fluid3d 0)

pico_add_extra_outputs(fluid3d)
