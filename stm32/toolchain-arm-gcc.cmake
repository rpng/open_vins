# Minimal ARM GCC toolchain for STM32H7
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_NONE_EABI "arm-none-eabi")
set(CMAKE_C_COMPILER ${ARM_NONE_EABI}-gcc)
set(CMAKE_CXX_COMPILER ${ARM_NONE_EABI}-g++)
set(CMAKE_ASM_COMPILER ${ARM_NONE_EABI}-gcc)

# Flags (adjust per H7 variant)
set(MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
set(COMMON_WARN "-Wall -Wextra -Werror=return-type")

set(CMAKE_C_FLAGS_INIT "${MCU_FLAGS} ${COMMON_WARN}")
set(CMAKE_CXX_FLAGS_INIT "${MCU_FLAGS} ${COMMON_WARN} -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -Wl,--gc-sections")

# Cache entries to make it easy to override
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
