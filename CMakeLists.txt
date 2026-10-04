# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(FloppyController C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(FloppyController
    src/main.cpp
    src/floppy_hw.cpp
    src/usb_msc.cpp
    src/usb_descriptors.cpp
    src/led_drv.cpp
    src/mfm_decoder.cpp
    src/mfm_encoder.cpp
    src/mfm_writer.cpp
    src/mode_config.cpp
    src/debug_serial.cpp
    src/gw/gw_proto.cpp
)

# Generate C++ headers from the PIO assembly files
pico_generate_pio_header(FloppyController ${CMAKE_CURRENT_LIST_DIR}/pio/flux_reader.pio)
pico_generate_pio_header(FloppyController ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)
pico_generate_pio_header(FloppyController ${CMAKE_CURRENT_LIST_DIR}/pio/flux_writer.pio)
pico_generate_pio_header(FloppyController ${CMAKE_CURRENT_LIST_DIR}/pio/gw_writer.pio)

pico_set_program_name(FloppyController "FloppyController")
pico_set_program_version(FloppyController "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(FloppyController 0)
pico_enable_stdio_usb(FloppyController 0)

# Add the standard library to the build
target_link_libraries(FloppyController 
    pico_stdlib 
    pico_multicore
    tinyusb_device 
    tinyusb_board
    hardware_pio
    hardware_clocks
    hardware_dma
)

# Add the standard include files to the build
target_include_directories(FloppyController PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
)

pico_add_extra_outputs(FloppyController)

