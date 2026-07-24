# Validation-gate results

Run date: 2026-07-22

## Gate 1 — ground-truth/IMU frame check

- Status: PASS
- Sequence: MH_01_easy
- Stationary window: 23.770–28.765 s
- Samples: 1,000
- Mean speed: 0.001163 m/s
- Mean vector discrepancy: 0.061442 m/s²
- RMSE: 0.039280 m/s²
- Axis MAE: [0.039891, 0.027558, 0.025373] m/s²

## Gate 2 — stock EuRoC ATE regression

- Status: PASS
- Sequences: 11/11
- Allowed ratio: 1.25× pinned reference
- Observed maximum ratio: 1.000×

The complete APE/RPE values are stored in [`../../openvins_benchmark/summary.csv`](../../openvins_benchmark/summary.csv).
