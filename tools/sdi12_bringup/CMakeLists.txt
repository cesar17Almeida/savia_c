cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(sdi12_bringup C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(sdi12_bringup main.c)
target_link_libraries(sdi12_bringup pico_stdlib)

# Serial output over USB CDC (the probe is on GP18, not the UART pins).
pico_enable_stdio_usb(sdi12_bringup 1)
pico_enable_stdio_uart(sdi12_bringup 0)

pico_add_extra_outputs(sdi12_bringup)
