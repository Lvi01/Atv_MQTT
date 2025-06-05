# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mqtt_client C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(mqtt_client mqtt_client.c 
                lib/ssd1306.c
                lib/font.h)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/final.pio)

pico_set_program_name(mqtt_client "mqtt_client")
pico_set_program_version(mqtt_client "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(mqtt_client 0)
pico_enable_stdio_usb(mqtt_client 1)

# Add the standard library to the build
target_link_libraries(mqtt_client
    pico_stdlib
    hardware_adc
    hardware_i2c
    hardware_timer
    hardware_pwm
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    pico_mbedtls
    pico_lwip_mbedtls
)

# Add the standard include files to the build
target_include_directories(mqtt_client PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(mqtt_client 
        
        )

pico_add_extra_outputs(mqtt_client)




