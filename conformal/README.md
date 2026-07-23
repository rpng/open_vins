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

The Stage-2 HDF5 loaders, variable-feature collation, four-channel Net-B target
derivation, sequence-disjoint validation loops, and reproducible checkpoints
are implemented. Target sidecars and trained checkpoints still need to be
materialized on the remote GPU server. The conformal calibration experiment
and Stage 3 corruption sweeps remain the next implementation block. The two
known theory/self-test discrepancies were intentionally left unchanged.

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

1. Generate and distribution-check all eleven Net-B target sidecars.
2. Train Net A and Net B and inspect held-out calibration NLL.
3. Fit one conformal quantile per modality without touching the test sequences.
4. Implement the Stage-3 sigma sidecar reader using the existing
   `set_msckf_sigma_provider()` and `set_imu_noises()` hooks.
5. Run A3 first, then C0; do not run the full corruption suite until those two
   de-risk experiments succeed.

See `changes.md` for the original experiment map. Treat its remaining scaffold
labels and the byte-for-byte-core claim as historical notes where they conflict
with this status section.
