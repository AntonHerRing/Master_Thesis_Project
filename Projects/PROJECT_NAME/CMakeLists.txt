# Generated Cmake ES Lab-Kit project file for RP2350, based on Pico project.

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
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Find the ES-Lab-Kit board files
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../boards)

set(PICO_BOARD es-lab-kit-V1A CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in the FreeRTOS kernel
set(FREERTOS_KERNEL_PATH "../../../FreeRTOS-Kernel")
include(FreeRTOS_Kernel_import.cmake)

project(PROJECT_NAME C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Create a variable with all BSP source files and print the list when running CMake.
file(GLOB BSP_SOURCES "../../bsp/*.c")
message(BSP_SOURCES="${BSP_SOURCES}")

# Create a variable with all Trace source files and print the list when running CMake.
file(GLOB TRACE_SOURCES "../../../Tools/RT_Trace/target/*.c")
message(TRACE_SOURCES="${TRACE_SOURCES}")

# Add executable. Default name is the project name, version 0.1
include_directories(../../bsp ../../../Tools/RT_Trace/target) # Add include files for the bsp
add_executable(PROJECT_NAME main.c ${BSP_SOURCES} ${TRACE_SOURCES})

pico_set_program_name(PROJECT_NAME "PROJECT_NAME")
pico_set_program_version(PROJECT_NAME "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PROJECT_NAME 1)
pico_enable_stdio_usb(PROJECT_NAME 0)

# Add the standard library to the build
target_link_libraries(PROJECT_NAME
        pico_stdlib
        pico_multicore
        hardware_spi
        hardware_i2c
        hardware_gpio
        hardware_pwm
        hardware_uart
        FreeRTOS-Kernel-Heap4)

# Add the standard include files to the build
target_include_directories(PROJECT_NAME PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(PROJECT_NAME)