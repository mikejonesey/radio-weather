cmake_minimum_required(VERSION 3.17)
# Including SDK before project def
include(pico_sdk_import.cmake)

# Define Radio Weather Project, change from C to C CXX ASM
##project(radio_weather C CXX ASM)
project(radio_weather C CXX ASM)

# Set C standard (dropper from 99 to 11)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add Source
add_executable(radio_weather main.c main.h bme280_i2c.c bme280_i2c.h lora_spi.c lora_spi.h misc.c rfm69hcw_spi.c rfm69hcw_spi.h common.c common.h)

# Add Pico SDK Includes
# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(radio_weather pico_stdlib hardware_i2c hardware_spi)

# DEBUG MODE START
# enable usb output, disable uart output
pico_enable_stdio_usb(radio_weather 1)
pico_enable_stdio_uart(radio_weather 0)
# DEBUG MODE END

# create map/bin/hex file etc.
pico_add_extra_outputs(radio_weather)

# odd url stuff
pico_set_program_url(radio_weather "https://gitlab.mj0.uk/pico/radio-weather/tree/HEAD")
