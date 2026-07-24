#!/usr/bin/env python3
"""Derive physically dimensioned Net-B supervision from Stage-1 and raw EuRoC.

The Stage-1 dump contains the causal 20x6 input window, but intentionally does
not invent a four-channel target.  This script creates one sidecar per sequence
using the exact raw IMU timestamps and consecutive motion-capture states.

For an interval of duration T, the four scalar targets are

  gyro white noise: RMS SO(3) integration error / sqrt(T)
  accel white noise: RMS velocity integration error / sqrt(T)
  gyro bias RW:     RMS increment of (estimated - GT) bias error / sqrt(T)
  accel bias RW:    RMS increment of (estimated - GT) bias error / sqrt(T)

EuRoC's supplied biases are piecewise constant, so their raw increments are not
usable random-walk supervision.  Bias-error increments instead quantify the
unmodelled evolution of the two filter bias states (including correction
jumps).  The acceleration integration uses interpolated ground-truth attitude
so its label is not contaminated by the gyro integration error.  Targets are
also divided by the four stock EuRoC densities.  Those dimensionless values are
what Net B learns because its output is a multiplicative correction to the
stock OpenVINS Q parameters.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import h5py
import numpy as np


SEQUENCES = (
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult",
    "MH_05_difficult", "V1_01_easy", "V1_02_medium", "V1_03_difficult",
    "V2_01_easy", "V2_02_medium", "V2_03_difficult",
)
TARGET_COLUMNS = ("sigma_w", "sigma_a", "sigma_wb", "sigma_ab")
BASE_SIGMAS = np.asarray((1.6968e-4, 2.0e-3, 1.9393e-5, 3.0e-3), dtype=np.float64)
GRAVITY = np.asarray((0.0, 0.0, 9.81), dtype=np.float64)


def _skew(v: np.ndarray) -> np.ndarray:
    return np.asarray(((0.0, -v[2], v[1]), (v[2], 0.0, -v[0]), (-v[1], v[0], 0.0)))


def _exp_so3(phi: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(phi))
    k = _skew(phi)
    if theta < 1e-8:
        return np.eye(3) + k + 0.5 * (k @ k)
    a = math.sin(theta) / theta
    b = (1.0 - math.cos(theta)) / (theta * theta)
    return np.eye(3) + a * k + b * (k @ k)


def _log_so3(rotation: np.ndarray) -> np.ndarray:
    cos_theta = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(cos_theta)
    vee = np.asarray((
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ))
    if theta < 1e-8:
        return 0.5 * vee
    return (0.5 * theta / math.sin(theta)) * vee


def _jpl_quat_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    """Match ov_core::quat_2_Rot (JPL, scalar last), returning R_GtoI."""
    q = np.asarray(q_xyzw, dtype=np.float64)
    q = q / np.linalg.norm(q)
    v, w = q[:3], float(q[3])
    return (2.0 * w * w - 1.0) * np.eye(3) - 2.0 * w * _skew(v) + 2.0 * np.outer(v, v)


def _nlerp_jpl(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    """Shortest-path normalized interpolation; 5 ms substeps make this sufficient."""
    q1_aligned = q1 if float(np.dot(q0, q1)) >= 0.0 else -q1
    q = (1.0 - alpha) * q0 + alpha * q1_aligned
    return q / np.linalg.norm(q)


def _read_imu(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", comments="#", dtype=np.float64)
    if data.ndim != 2 or data.shape[1] != 7:
        raise ValueError(f"{path}: expected seven IMU CSV columns, got {data.shape}")
    return data[:, 0] * 1e-9, data[:, 1:7]


def _find_imu_csv(dataset_root: Path, sequence: str) -> Path:
    # The server has a flat /data/<sequence>/mav0 layout and can also retain a
    # nested local-style copy. Prefer the flat canonical path when both exist.
    direct = dataset_root / sequence / "mav0" / "imu0" / "data.csv"
    if direct.is_file():
        return direct
    matches = sorted(dataset_root.glob(f"*/{sequence}/{sequence}/mav0/imu0/data.csv"))
    if len(matches) != 1:
        raise FileNotFoundError(
            f"expected exactly one raw IMU CSV for {sequence} below {dataset_root}, got {matches}"
        )
    return matches[0]


def _interpolated_interval(
    timestamps: np.ndarray, measurements: np.ndarray, t0: float, t1: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return samples over [t0,t1], adding linearly interpolated endpoints."""
    if not (timestamps[0] <= t0 < t1 <= timestamps[-1]):
        raise ValueError(f"interval [{t0}, {t1}] lies outside raw IMU range")
    lo = int(np.searchsorted(timestamps, t0, side="right"))
    hi = int(np.searchsorted(timestamps, t1, side="left"))
    inner_t = timestamps[lo:hi]
    t = np.concatenate(([t0], inner_t, [t1]))
    values = np.empty((len(t), 6), dtype=np.float64)
    for channel in range(6):
        values[:, channel] = np.interp(t, timestamps, measurements[:, channel])
    # Exact camera/IMU timestamp coincidences can duplicate an endpoint.
    keep = np.concatenate(([True], np.diff(t) > 1e-9))
    return t[keep], values[keep]


def _derive_interval(
    imu_t: np.ndarray,
    imu_x: np.ndarray,
    t0: float,
    t1: float,
    gt0: np.ndarray,
    gt1: np.ndarray,
    state0: np.ndarray,
    state1: np.ndarray,
) -> tuple[np.ndarray, int]:
    sample_t, sample_x = _interpolated_interval(imu_t, imu_x, t0, t1)
    total_dt = t1 - t0
    q0, q1 = gt0[1:5], gt1[1:5]
    v0, v1 = gt0[8:11], gt1[8:11]
    bg0, bg1 = gt0[11:14], gt1[11:14]
    ba0, ba1 = gt0[14:17], gt1[14:17]

    rotation_pred = _jpl_quat_to_rot(q0)
    velocity_pred = v0.copy()
    for j in range(len(sample_t) - 1):
        dt = float(sample_t[j + 1] - sample_t[j])
        midpoint = 0.5 * (sample_t[j] + sample_t[j + 1])
        alpha = float(np.clip((midpoint - t0) / total_dt, 0.0, 1.0))
        measurement = 0.5 * (sample_x[j] + sample_x[j + 1])
        bg = (1.0 - alpha) * bg0 + alpha * bg1
        ba = (1.0 - alpha) * ba0 + alpha * ba1

        omega = measurement[:3] - bg
        rotation_pred = _exp_so3(-omega * dt) @ rotation_pred

        # Use GT orientation to isolate the accelerometer channel from gyro error.
        rotation_gt_mid = _jpl_quat_to_rot(_nlerp_jpl(q0, q1, alpha))
        specific_force_global = rotation_gt_mid.T @ (measurement[3:] - ba)
        velocity_pred += (specific_force_global - GRAVITY) * dt

    rotation_gt1 = _jpl_quat_to_rot(q1)
    rotation_error = _log_so3(rotation_gt1 @ rotation_pred.T)
    velocity_error = v1 - velocity_pred
    scale = math.sqrt(3.0 * total_dt)
    bg_error_increment = (state1[10:13] - bg1) - (state0[10:13] - bg0)
    ba_error_increment = (state1[13:16] - ba1) - (state0[13:16] - ba0)
    physical_density = np.asarray((
        np.linalg.norm(rotation_error) / scale,
        np.linalg.norm(velocity_error) / scale,
        np.linalg.norm(bg_error_increment) / scale,
        np.linalg.norm(ba_error_increment) / scale,
    ))
    return physical_density, len(sample_t)


def derive_sequence(
    stage1_path: Path, imu_csv: Path, output_path: Path, overwrite: bool
) -> dict[str, object]:
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"{output_path} exists (pass --overwrite to replace it)")
    imu_t, imu_x = _read_imu(imu_csv)
    with h5py.File(stage1_path, "r") as source:
        frame_t = source["/frames/timestamp"][:].astype(np.float64)
        gt = source["/frames/groundtruth"][:].astype(np.float64)
        state = source["/frames/state"][:].astype(np.float64)
        windows = source["/frames/imu_window"][:].astype(np.float32)
        sequence_attr = source["/meta"].attrs.get("sequence", stage1_path.name.split("_stage1")[0])
        if isinstance(sequence_attr, bytes):
            sequence_attr = sequence_attr.decode()

    n = len(frame_t)
    physical = np.full((n, 4), np.nan, dtype=np.float64)
    interval_dt = np.full(n, np.nan, dtype=np.float64)
    interval_samples = np.zeros(n, dtype=np.int32)
    valid = np.zeros(n, dtype=np.uint8)
    for i in range(1, n):
        dt = float(frame_t[i] - frame_t[i - 1])
        if not (0.0 < dt <= 0.25) or not np.isfinite(gt[i - 1:i + 1]).all():
            continue
        try:
            physical[i], interval_samples[i] = _derive_interval(
                imu_t, imu_x, frame_t[i - 1], frame_t[i], gt[i - 1], gt[i],
                state[i - 1], state[i],
            )
        except ValueError:
            continue
        interval_dt[i] = dt
        valid[i] = int(np.isfinite(physical[i]).all())

    normalized = physical / BASE_SIGMAS[None, :]
    with h5py.File(output_path, "w") as target:
        meta = target.create_group("meta")
        meta.attrs["schema_version"] = "1"
        meta.attrs["sequence"] = str(sequence_attr)
        meta.attrs["source_stage1"] = stage1_path.name
        meta.attrs["source_imu_csv"] = str(imu_csv)
        meta.attrs["target_columns"] = ",".join(TARGET_COLUMNS)
        meta.attrs["physical_units"] = (
            "rad/s/sqrt(Hz),m/s^2/sqrt(Hz),rad/s^2/sqrt(Hz),m/s^3/sqrt(Hz)"
        )
        meta.attrs["method"] = (
            "RMS 3-axis GT preintegration residual divided by sqrt(interval); "
            "accelerometer residual uses interpolated GT attitude; random-walk "
            "targets use increments of OpenVINS-minus-GT bias error"
        )
        meta.create_dataset("base_sigmas", data=BASE_SIGMAS)
        target.create_dataset("frame_timestamp", data=frame_t)
        target.create_dataset("imu_window", data=windows, compression="gzip", compression_opts=4)
        target.create_dataset("target_physical_density", data=physical)
        target.create_dataset("target_normalized", data=normalized)
        target.create_dataset("interval_dt", data=interval_dt)
        target.create_dataset("interval_sample_count", data=interval_samples)
        target.create_dataset("valid", data=valid)

    valid_rows = physical[valid.astype(bool)]
    summary: dict[str, object] = {
        "sequence": str(sequence_attr),
        "frames": n,
        "valid_targets": int(valid.sum()),
        "median_interval_s": float(np.nanmedian(interval_dt)),
    }
    for j, name in enumerate(TARGET_COLUMNS):
        summary[f"{name}_median"] = float(np.median(valid_rows[:, j]))
        summary[f"{name}_p95"] = float(np.quantile(valid_rows[:, j], 0.95))
        summary[f"{name}_normalized_median"] = float(np.median(valid_rows[:, j] / BASE_SIGMAS[j]))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage1-dir", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sequences", nargs="+", default=list(SEQUENCES))
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    summaries = []
    for sequence in args.sequences:
        source = args.stage1_dir / f"{sequence}_stage1.h5"
        output = args.out_dir / f"{sequence}_stage2_netb.h5"
        summary = derive_sequence(
            source, _find_imu_csv(args.dataset_root, sequence), output, args.overwrite
        )
        summaries.append(summary)
        print(json.dumps(summary, sort_keys=True))

    summary_path = args.out_dir / "netb_target_summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    print(f"wrote {len(summaries)} sidecars and {summary_path}")


if __name__ == "__main__":
    main()
