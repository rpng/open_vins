#include "ov_runtime.h"

// TODO: include minimal headers from ov_core/ov_msckf when integrating

void ov_runtime_init(void) {
    // Initialize buffers, state, sensors
    // Replace with actual init from ov_init and ov_msckf
}

void ov_runtime_step(void) {
    // Read IMU sample(s), camera frame
    // Run propagation + update (MSCKF)
    // Output pose/velocity if needed
}
