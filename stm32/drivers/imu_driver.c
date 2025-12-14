#include <stdint.h>
// Placeholder IMU driver: replace with STM32 HAL or LL

void imu_init(void) {
    // setup SPI/I2C, DMA, interrupts
}

int imu_read(int16_t *gx, int16_t *gy, int16_t *gz,
             int16_t *ax, int16_t *ay, int16_t *az) {
    // read registers into provided buffers
    return 0; // return 0 on success
}
