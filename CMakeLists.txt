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

cmake_minimum_required(VERSION 3.13)

# Incluir funciones del SDK
include(pico_sdk_import.cmake)

project(rp2040droneUpdated C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Inicializar el SDK
pico_sdk_init()

add_executable(rp2040droneUpdated
    rp2040droneUpdated.c
)

# --- GENERACIÓN DE CÓDIGO PIO ---
# Esto genera los archivos .pio.h a partir de tus archivos .pio
pico_generate_pio_header(rp2040droneUpdated ${CMAKE_CURRENT_LIST_DIR}/pio_ppm_rx.pio)
pico_generate_pio_header(rp2040droneUpdated ${CMAKE_CURRENT_LIST_DIR}/pio_dshot.pio)

# --- LIBRERÍAS DE HARDWARE ---
# Aquí es donde arreglamos tu error de "hardware/spi.h"
target_link_libraries(rp2040droneUpdated
    pico_stdlib 
    hardware_spi 
    hardware_pio 
    hardware_clocks
)

# Habilitar salida USB para depuración (printf)
pico_enable_stdio_usb(rp2040droneUpdated 1)
pico_enable_stdio_uart(rp2040droneUpdated 0)

# Crear archivos finales (hex, bin, uf2)
pico_add_extra_outputs(rp2040droneUpdated)