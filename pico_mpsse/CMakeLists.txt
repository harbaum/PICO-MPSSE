cmake_minimum_required(VERSION 3.13...3.27)

set(PROJECT pico_mpsse)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_mpsse)

pico_sdk_init()

add_executable(${PROJECT}
        ${PROJECT}.c
	pio_jtag.c
)

pico_generate_pio_header(${PROJECT} ${CMAKE_CURRENT_LIST_DIR}/jtag.pio)

target_link_libraries(${PROJECT} PRIVATE
     hardware_pio
     hardware_dma
     pico_stdlib
     pico_unique_id
)

# Add extra outputs for the "test" target (e.g., UF2 file)
pico_add_extra_outputs(${PROJECT})
pico_enable_stdio_usb(${PROJECT} 0)
pico_enable_stdio_uart(${PROJECT} 1)

add_custom_target(flash
    COMMAND echo "Flashing ${PROJECT} ..."
    COMMAND openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ${PROJECT}.elf verify reset exit"
    DEPENDS "${PROJECT}"
    COMMENT "Flash target using openocd"
)

add_custom_target(reset
    COMMAND echo "Reseting ${PROJECT} ..."
    COMMAND openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c init -c reset -c exit
    COMMENT "Reset target using openocd"
)

add_custom_target(term
    COMMAND term.sh /dev/ttyACM0 115200
)

