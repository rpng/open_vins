# STM32H7 Port Skeleton

This branch provides a minimal skeleton to begin porting OpenVINS to STM32H7.

Goals:
- Lightweight runtime wrapping `ov_msckf` with static allocations.
- HAL/driver placeholders for IMU and camera.
- Toolchain file for ARM GCC and a minimal CMake target.

Directory layout:
- `toolchain-arm-gcc.cmake`: CMake toolchain for ARM GCC.
- `CMakeLists.txt`: Minimal build configuration for MCU target.
- `src/main.c`: Entry point with basic init loop.
- `src/ov_runtime.c`: Thin wrapper around algorithm steps.
- `include/ov_runtime.h`: Interface for runtime.
- `drivers/imu_driver.c`: IMU stub.
- `drivers/cam_driver.c`: Camera stub.

Build (example):
1. Install ARM GCC (e.g., `gcc-arm-none-eabi`).
2. Configure:
   ```bash
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=toolchain-arm-gcc.cmake -DMCU_FPU=ON
   cmake --build build
   ```
3. Linker script and startup files must be added per your STM32H7 board.

Notes:
- Replace stubs with actual HAL drivers (STM32CubeH7 or LL).
- Avoid dynamic allocations; prefer static buffers.
- Integrate FreeRTOS if you need scheduling.
