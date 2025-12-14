#include <stdint.h>
// Placeholder Camera driver: replace with DCMI/CSI + DMA

void cam_init(void) {
    // configure interface, resolution, pixel format
}

int cam_capture_frame(uint8_t *buffer, uint32_t max_len) {
    // fill buffer with captured frame
    return 0; // bytes captured or 0 on failure
}
