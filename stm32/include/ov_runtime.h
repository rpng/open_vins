#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// Minimal runtime interface for OpenVINS MSCKF on MCU
void ov_runtime_init(void);
void ov_runtime_step(void); // process one IMU+camera cycle

#ifdef __cplusplus
}
#endif
