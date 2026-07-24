# EuRoC results: conformal OpenVINS versus popular systems

## Bottom line

The unmodified OpenVINS benchmark remains the strongest result produced by
this repository: its mean ATE RMSE over all 11 EuRoC sequences is **0.1037 m**.
The final causal learned-noise and conformalised arms are negative results:
their mean ATE rises to **0.2821 m** and **0.3265 m**, respectively.

Against published stereo-inertial results, stock OpenVINS is better than the
reported VINS-Fusion mean, but worse than OKVIS2 VIO, causal OKVIS2 SLAM, and
ORB-SLAM3. The learned and conformalised arms are worse than every external
method in this comparison.

## What each experimental stage established

| Stage | Output | Main result | Trajectory comparison? |
|---|---|---|---|
| Stock baseline | OpenVINS stereo-inertial on all EuRoC sequences | Mean ATE RMSE 0.1037 m; mean 1 m RPE RMSE 0.0365 m | Yes |
| Stage 1 | Validated HDF5 diagnostic corpus | 11/11 sequences valid; 23,302 frames and 74,329 pre-gate feature candidates | No new method; stock data collection |
| Stage 2A | Visual Net A | Accepted checkpoint: training NLL 1.04437, frozen-calibration NLL 1.14010 | No |
| Stage 2B | Inertial Net B | Conditional TCN rejected; only epoch-zero constant-Q correction retained, calibration NLL 3.26105 | No |
| Stage 2 conformal | Fixed 90% calibration on MH04 and V1_03 | Visual coverage 83.67% → 90.02%, `q=1.170186`; inertial coverage 84.66% → 90.01%, `q=1.237830` | No |
| Stage 3 stock | Causal Stage-3 runner with learned inference disabled | Mean ATE RMSE 0.1112 m | Yes |
| Stage 3 learned | Causal live Net-A visual sigma | Mean ATE RMSE 0.2821 m; near-100% gate acceptance | Yes; rejected |
| Stage 3 conformalised | Live Net A multiplied by fixed conformal scale | Mean ATE RMSE 0.3265 m; near-100% gate acceptance | Yes; rejected |

Stages 1 and 2 should not be assigned ATE values: they produce diagnostics,
models, and calibration guarantees, not independent trajectory estimators.

## Mean ATE RMSE over all 11 sequences

Lower is better.

| Method | Mode | Mean ATE RMSE (m) | Relative to Stage-3 stock | Result provenance |
|---|---|---:|---:|---|
| ORB-SLAM3 | Stereo-inertial SLAM | **0.0348** | 0.313× | Published; median of 10 runs |
| OKVIS2 | Causal stereo-inertial SLAM | **0.0481** | 0.432× | Published; loop closure enabled |
| OKVIS2 | Stereo-inertial VIO | **0.0709** | 0.637× | Published; closest external architecture |
| OpenVINS | Stock, full-sequence local benchmark | **0.1037** | 0.932× | Measured locally |
| OpenVINS | Stage-3 stock | **0.1112** | 1.000× | Measured locally |
| VINS-Fusion | Stereo-inertial | **0.1381** | 1.241× | Published comparison run |
| OpenVINS | Stage-3 learned | **0.2821** | 2.536× | Measured locally |
| OpenVINS | Stage-3 conformalised | **0.3265** | 2.935× | Measured locally |

The full-sequence stock OpenVINS mean is about 25% lower than the reported
VINS-Fusion mean. ORB-SLAM3 is about 3.0× lower than full-sequence stock
OpenVINS, while OKVIS2 VIO is about 1.46× lower. These comparisons are useful
context, not a controlled ranking, because the external values were not
regenerated in our container on the same machine.

## Per-sequence ATE RMSE

All values are metres. `OV` means OpenVINS. `OK2 causal` includes causal loop
closure; `OK2 VIO` does not.

| Sequence | OV full stock | S3 stock | S3 learned | S3 conformal | ORB-SLAM3 | VINS-Fusion | OK2 VIO | OK2 causal |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| MH01 | 0.0908 | 0.0788 | 0.2588 | 0.3225 | 0.036 | 0.166 | 0.057 | 0.044 |
| MH02 | 0.1244 | 0.0965 | 0.3323 | 0.4066 | 0.033 | 0.152 | 0.044 | 0.036 |
| MH03 | 0.1376 | 0.1947 | 0.3531 | 0.3942 | 0.035 | 0.125 | 0.082 | 0.050 |
| MH04 | 0.1661 | 0.1751 | 0.6525 | 0.7509 | 0.051 | 0.280 | 0.189 | 0.089 |
| MH05 | 0.2428 | 0.2780 | 0.8561 | 0.8907 | 0.082 | 0.284 | 0.141 | 0.112 |
| V101 | 0.0546 | 0.0565 | 0.0846 | 0.0963 | 0.038 | 0.076 | 0.043 | 0.037 |
| V102 | 0.0573 | 0.0706 | 0.1571 | 0.1566 | 0.014 | 0.069 | 0.037 | 0.021 |
| V103 | 0.0595 | 0.0600 | 0.0827 | 0.1321 | 0.024 | 0.114 | 0.036 | 0.035 |
| V201 | 0.0589 | 0.0556 | 0.1108 | 0.1312 | 0.032 | 0.066 | 0.044 | 0.036 |
| V202 | 0.0452 | 0.0378 | 0.0811 | 0.1416 | 0.014 | 0.091 | 0.044 | 0.024 |
| V203 | 0.1033 | 0.1201 | 0.1340 | 0.1681 | 0.024 | 0.096 | 0.063 | 0.045 |
| **Mean** | **0.1037** | **0.1112** | **0.2821** | **0.3265** | **0.0348** | **0.1381** | **0.0709** | **0.0481** |

The machine-readable version is
[`euroc_ate_comparison.csv`](euroc_ate_comparison.csv).

## Held-out test split

The scientifically primary conformal result is the predeclared held-out split
MH05, V202, and V203:

| Arm | Mean ATE RMSE (m) | Mean 1 s RPE RMSE (m) | Mean gate pass rate |
|---|---:|---:|---:|
| Stage-3 stock | 0.1453 | 0.0405 | 74.07% |
| Stage-3 learned | 0.3571 | 0.0620 | 99.96% |
| Stage-3 conformalised | 0.4002 | 0.0712 | 100.00% |

Net A frequently reaches `sigma_pix=exp(7)=1096.63`; conformal scaling raises
the maximum to 1283.27. This nearly disables chi-squared rejection and explains
why calibration coverage improved in Stage 2 while trajectory accuracy became
worse in Stage 3. Marginal calibration is not the same as downstream estimator
performance.

## Comparability and protocol

- All rows use translational ATE RMSE in metres and all 11 standard EuRoC MAV
  sequences.
- The local full-sequence stock benchmark uses EVO, SE(3) Umeyama alignment,
  and a 0.02 s association limit.
- Stage 3 uses rigid SE(3) position alignment on its logged, offset-trimmed
  trajectories. Its stock arm is the correct within-experiment control.
- ORB-SLAM3 reports SE(3)-aligned results and gives the median of 10 executions.
  The VINS-Fusion values in the same table were obtained by the ORB-SLAM3
  authors using the public code and default configuration.
- OKVIS2 reports position-and-yaw alignment. Its causal SLAM arm can use loop
  closures observed up to the current time; its VIO arm is the closer
  comparison to OpenVINS. The lower non-causal OKVIS2 result is deliberately
  excluded because it uses future loop-closure information.
- ORB-SLAM3 and OKVIS2 SLAM are smoothing/mapping systems with loop closure;
  OpenVINS is a causal filtering VIO. The table should not be presented as a
  same-hardware, same-codebase head-to-head benchmark.
- Only MH05, V202, and V203 are untouched test sequences for the learned
  conformal experiment. The all-11 mean is supplementary context because it
  includes training and calibration sequences.

## Sources

- C. Campos et al., “ORB-SLAM3: An Accurate Open-Source Library for Visual,
  Visual-Inertial and Multi-Map SLAM,” Table II:
  <https://arxiv.org/pdf/2007.11898>
- S. Leutenegger, “OKVIS2: Realtime Scalable Visual-Inertial SLAM with Loop
  Closure,” Table I: <https://arxiv.org/pdf/2202.09199>
- VINS-Fusion official implementation:
  <https://github.com/HKUST-Aerial-Robotics/VINS-Fusion>
- Local OpenVINS benchmark: [`../../../openvins_benchmark/summary.csv`](../../../openvins_benchmark/summary.csv)
- Local Stage-3 report:
  [`../stage3_online_all11/reports/stage3_benchmark_summary.csv`](../stage3_online_all11/reports/stage3_benchmark_summary.csv)
