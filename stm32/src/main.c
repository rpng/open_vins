#include <stdint.h>
#include <stdbool.h>
#include "ov_runtime.h"

// Basic MCU entry stub (replace with actual startup and HAL init)
int main(void) {
    // HAL_Init(); SystemClock_Config(); Board init...
    ov_runtime_init();

    while (1) {
        ov_runtime_step();
        // sleep or wait for next sensor event
    }

    return 0;
}
