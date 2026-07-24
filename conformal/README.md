# Conformal Q/R experiments for OpenVINS

This directory contains the experiment pipeline for learning and conformalising
the OpenVINS process noise **Q** and measurement noise **R**.

## Current implementation status

The experiment foundation is implemented and verified:

- Gate 1 parses EuRoC/ASL transforms, finds a stationary segment, and checks
  ground-truth-derived specific force against the IMU.
- Gate 2 compares all 11 stock EuRoC ATE results with the pinned baseline in
  `validation_gates/openvins_euroc_reference.csv`.
- `stage1_dumps/run_asl_msckf.cpp` reads raw ASL stereo/IMU streams without a
  rosbag, preserves IMU-before-camera ordering, and drives OpenVINS.
- `DiagnosticsLogger` writes extendible HDF5 datasets for states, 15×15 IMU
  covariance, ground truth, 20×6 IMU windows, tracker context, and feature
  diagnostics captured before chi-squared rejection.
- Estimator landmarks are aligned from the OpenVINS local world frame to the
  EuRoC mocap frame before GT reprojection targets are calculated.
- Optional core hooks expose pre-gate MSCKF diagnostics, per-feature pixel
  sigma, and runtime IMU noise replacement. With no provider/callback installed,
  the stock MSCKF numerical path is unchanged.
- `stage1_dumps/validate_dump.py` rejects incomplete or numerically invalid
  dumps, including dumps with insufficient finite GT reprojection targets.

Stages 1--3 have now been materialized and validated on the remote server.
Stage 2 accepted a geometry-valid Net A and, as a negative result, only the
epoch-zero constant-Q Net-B baseline. Fixed calibration on MH04 and V1_03 gave
visual `q_alpha=1.170186` and inertial `q_alpha=1.237830`.

The final Stage-3 benchmark uses causal live Net-A inference inside each MSCKF
update; it does not reuse feature IDs or observations from a different filter
trajectory. All 33 runs (11 sequences × stock/learned/conformalised) are
downloaded under `results/stage3_online_all11/` and pass the supplied
checksums. On the three held-out test sequences, mean ATE RMSE is 0.145 m for
stock, 0.357 m for learned, and 0.400 m for conformalised. Mean relative ATE is
therefore 2.11× and 2.79× stock. The learned noise makes the feature gate pass
almost every candidate (99.96% versus 74.07% stock), so this is a clear
negative result for the present target/model rather than evidence of improved
robustness. The two known theory/self-test discrepancies remain intentionally
unchanged and were not used as experiment gates.

The optional hooks require small changes under `ov_msckf/`; therefore the old
claim in `changes.md` that the OpenVINS core is byte-for-byte untouched is no
longer literally true. The defensible claim is narrower: the hooks are opt-in,
and stock runs install none of them.

## Validate the foundation

From the repository root:

```bash
python3 conformal/validation_gates/gate1_groundtruth_frame_check.py \
  EuRoC_MAV/machine_hall/MH_01_easy/MH_01_easy

conformal/validation_gates/gate2_reproduce_euroc_ate.sh
```

Gate 2 reads `openvins_benchmark/summary.csv` by default. Set
`RUN_BENCHMARK=1` to regenerate the stock results before comparing them.

## Build Stage 1

The reproducible image builds only the Stage-1 package and its dependencies:

```bash
docker build -f conformal/Dockerfile.stage1 \
  -t openvins-conformal:stage1 .
```

## Generate one EuRoC dump

Inside the built image, the runner interface is:

```text
run_asl_msckf CONFIG_YAML SEQUENCE_DIR SEQUENCE_NAME OUTPUT_H5 [START_OFFSET_S]
```

Example container command:

```bash
docker run --rm \
  -v /data/MH_01_easy:/datasets/MH_01_easy:ro \
  -v /data/conformal_dumps:/results \
  openvins-conformal:stage1 \
  /catkin_ws/devel/lib/conformal_stage1/run_asl_msckf \
  /catkin_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
  /datasets/MH_01_easy MH_01_easy /results/MH_01_easy_stage1.h5 40
```

Validate it before training:

```bash
docker run --rm -v /data/conformal_dumps:/results:ro \
  openvins-conformal:stage1 \
  python3 /catkin_ws/src/open_vins/conformal/stage1_dumps/validate_dump.py \
  /results/MH_01_easy_stage1.h5
```

The complete verified EuRoC suite is downloaded under `results/stage1/`. Across all eleven
sequences it contains 23,302 frames and 74,329 pre-gate feature candidates. See
`results/stage1/stage1_suite_summary.csv` for per-sequence metrics and checksums.

## Build the Net-B target sidecars

The immutable Stage-1 files omit raw IMU timestamps. Rather than approximating
them, `derive_netb_targets.py` aligns each logged frame with the original 200 Hz
EuRoC CSV. It stores physical density proxies and their dimensionless ratios to
the stock OpenVINS densities:

```bash
python3 conformal/stage2_train/derive_netb_targets.py \
  --stage1-dir conformal/results/stage1 \
  --dataset-root EuRoC_MAV \
  --out-dir conformal/results/stage2/netb_targets
```

Net B is trained on the dimensionless ratios because its Stage-3 output is a
multiplicative correction `stock_sigma * exp(delta)`. The acceleration label
uses interpolated GT attitude to avoid folding gyro integration error into the
accelerometer channel. Because EuRoC's provided bias fields are largely
piecewise constant, the random-walk labels use increments of the OpenVINS bias
estimation error `(b_est - b_gt)`, not the nearly-zero raw GT bias increments.

For a PyTorch container without HDF5 bindings, export portable arrays first:

```bash
python3 conformal/stage2_train/export_stage2_npz.py \
  --stage1-dir conformal/results/stage1 \
  --netb-sidecar-dir conformal/results/stage2/netb_targets \
  --out-dir conformal/results/stage2/training_arrays
```

Net-A export rejects GT reprojection targets above the 752×480 image diagonal.
Such a value proves the reconstructed landmark's GT projection is outside the
image and is therefore an invalid supervision target, not a difficult visual
measurement. The threshold is geometry-derived and fixed before training.

## What to do next

Do not tune the current Net A on MH05, V2_02, or V2_03: they are the held-out
test set. The next defensible experiment is a new, predeclared visual-noise
target/model design using only the training sequences, with MH04 and V1_03
reserved for calibration/model selection. First diagnose the saturation
visible in Stage 3 (`sigma_pix_max≈exp(7)` and near-100% gate acceptance), then
retrain and repeat the causal live benchmark. Do not proceed to corruption
sweeps with the currently rejected model.

See `changes.md` for the original experiment map. Treat its remaining scaffold
labels and the byte-for-byte-core claim as historical notes where they conflict
with this status section.
