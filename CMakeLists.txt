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
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(synth C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(pico_audio_i2s_32b/src/pico_audio_32b pico_audio_32b)
add_subdirectory(pico_audio_i2s_32b pico_audio_i2s_32b)

# Add executable. Default name is the project name, version 0.1

add_executable(synth synth.c )

pico_set_program_name(synth "synth")
pico_set_program_version(synth "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(synth 0)
pico_enable_stdio_usb(synth 1)

# Add the standard library to the build
target_link_libraries(synth
    pico_stdlib
)

# Add the standard include files to the build
target_include_directories(synth PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(synth 
    hardware_dma
    pico_audio_32b
    pico_audio_i2s_32b
)

# target_compile_definitions(${PROJECT_NAME} PRIVATE
#    PICO_AUDIO_I2S_PIO=1
#    PICO_AUDIO_I2S_DMA_IRQ=1
#    CORE1_PROCESS_I2S_CALLBACK
# )

pico_add_extra_outputs(synth)

