cmake_minimum_required(VERSION 3.15)
project(openvins_stm32h7 C CXX ASM)

# Options
option(MCU_FPU "Enable hard FPU" ON)

# Sources
add_executable(openvins_stm32
    src/main.c
    src/ov_runtime.c
    drivers/imu_driver.c
    drivers/cam_driver.c
)

target_include_directories(openvins_stm32 PRIVATE include)

# Linker flags and memory map should be adjusted per board
# target_link_options(openvins_stm32 PRIVATE -TSTM32H7xx.ld)

# TODO: link to minimal OpenVINS core sources when selected
