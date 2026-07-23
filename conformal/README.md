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

Stage 2 training, the conformal calibration experiment, and Stage 3 corruption
sweeps remain scaffolds and are the next implementation block. The two known
theory/self-test discrepancies were intentionally left unchanged for now.

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

## What to do next

1. Implement `stage2_train/hdf5_dump_dataset.py` against the documented HDF5
   columns, using sequence-disjoint train/calibration/test splits.
2. Define and dump a defensible Net-B preintegration-error target; the current Stage-1 schema
   contains IMU windows but does not yet encode four channel-specific target errors.
3. Train Net A and Net B, then fit one conformal quantile per modality.
4. Implement the Stage-3 sigma sidecar reader using the existing
   `set_msckf_sigma_provider()` and `set_imu_noises()` hooks.
5. Run A3 first, then C0; do not run the full corruption suite until those two
   de-risk experiments succeed.

See `changes.md` for the original experiment map. Treat its remaining scaffold
labels and the byte-for-byte-core claim as historical notes where they conflict
with this status section.
