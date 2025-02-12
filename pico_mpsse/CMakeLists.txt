cmake_minimum_required(VERSION 3.13...3.27)

set(PROJECT pico_mpsse)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_mpsse)

pico_sdk_init()

add_executable(${PROJECT}
        ${PROJECT}.c
)

target_link_libraries(${PROJECT} PRIVATE pico_stdlib)
target_link_libraries(${PROJECT} PRIVATE pico_unique_id)

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

