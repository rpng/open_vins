#!/usr/bin/env python3
"""Gate 1: verify EuRoC ground-truth and IMU frame conventions.

The gate finds a low-speed window, transforms the ground-truth sensor pose into
the IMU frame using the two ASL ``T_BS`` matrices, and compares measured
accelerometer samples with the specific force predicted from ground truth.
It exits non-zero when the detected window is not stationary or the mean vector
discrepancy exceeds the configured tolerance.
"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])


@dataclass(frozen=True)
class Gate1Result:
    passed: bool
    start_s: float
    end_s: float
    mean_speed_mps: float
    mean_error_mps2: float
    rmse_mps2: float
    axis_mae_mps2: np.ndarray
    samples: int


def _load_csv(path: Path, min_columns: int) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open(newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < min_columns:
                raise ValueError(f"{path}: expected at least {min_columns} columns")
            rows.append([float(value) for value in row])
    if not rows:
        raise ValueError(f"{path}: no data rows")
    return np.asarray(rows, dtype=float)


def _load_t_bs(path: Path) -> np.ndarray:
    """Read the 4x4 ``T_BS.data`` matrix without requiring PyYAML.

    ASL sensor YAML begins with OpenCV's ``%YAML:1.0`` directive, which is not
    accepted by every generic YAML parser. The matrix is the only field needed
    here, so a small strict parser keeps Gate 1 dependency-free.
    """
    text = path.read_text()
    match = re.search(r"T_BS\s*:.*?data\s*:\s*\[([^]]+)\]", text, re.DOTALL)
    if match is None:
        raise ValueError(f"{path}: T_BS.data not found")
    values = [float(value) for value in match.group(1).replace("\n", " ").split(",")]
    if len(values) != 16:
        raise ValueError(f"{path}: T_BS.data has {len(values)} values, expected 16")
    transform = np.asarray(values, dtype=float).reshape(4, 4)
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{path}: invalid homogeneous transform")
    return transform


def _quat_wxyz_to_rotation(quaternions: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternions, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    rotations = np.empty((len(q), 3, 3), dtype=float)
    rotations[:, 0, 0] = 1 - 2 * (y * y + z * z)
    rotations[:, 0, 1] = 2 * (x * y - z * w)
    rotations[:, 0, 2] = 2 * (x * z + y * w)
    rotations[:, 1, 0] = 2 * (x * y + z * w)
    rotations[:, 1, 1] = 1 - 2 * (x * x + z * z)
    rotations[:, 1, 2] = 2 * (y * z - x * w)
    rotations[:, 2, 0] = 2 * (x * z - y * w)
    rotations[:, 2, 1] = 2 * (y * z + x * w)
    rotations[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return rotations


def _smooth(values: np.ndarray, samples: int) -> np.ndarray:
    samples = max(1, int(samples))
    if samples % 2 == 0:
        samples += 1
    if samples == 1:
        return values.copy()
    pad = samples // 2
    kernel = np.full(samples, 1.0 / samples)
    return np.stack(
        [np.convolve(np.pad(values[:, axis], (pad, pad), mode="edge"), kernel, mode="valid")
         for axis in range(values.shape[1])],
        axis=1,
    )


def _stationary_window(t_rel: np.ndarray, velocity: np.ndarray, duration_s: float) -> tuple[float, float]:
    dt = float(np.median(np.diff(t_rel)))
    count = max(2, int(round(duration_s / dt)))
    if count >= len(t_rel):
        raise ValueError("stationary-window duration exceeds the ground-truth trajectory")
    speed = np.linalg.norm(velocity, axis=1)
    moving_mean = np.convolve(speed, np.full(count, 1.0 / count), mode="valid")
    start = int(np.argmin(moving_mean))
    return float(t_rel[start]), float(t_rel[start + count - 1])


def acceleration_from_groundtruth(velocity_gt: np.ndarray, t_gt: np.ndarray,
                                  smoothing_s: float = 0.5) -> np.ndarray:
    """Differentiate smoothed GT velocity to obtain world-frame acceleration."""
    dt = float(np.median(np.diff(t_gt)))
    velocity_smooth = _smooth(np.asarray(velocity_gt), max(1, round(smoothing_s / dt)))
    return np.gradient(velocity_smooth, t_gt, axis=0)


def predicted_specific_force(a_world: np.ndarray, rotation_imu_to_world: np.ndarray,
                             bias_accel: np.ndarray) -> np.ndarray:
    """Predict accelerometer output: ``R_IW (a_W - g_W) + b_a``."""
    return np.einsum("nji,nj->ni", rotation_imu_to_world, a_world - GRAVITY) + bias_accel


def check_gate1(seq_dir: str | Path, static_window_s: tuple[float, float] | None = None,
                tol_mps2: float = 0.3, duration_s: float = 5.0,
                max_static_speed_mps: float = 0.05,
                plot_path: str | Path | None = None) -> Gate1Result:
    seq = Path(seq_dir)
    imu = _load_csv(seq / "mav0/imu0/data.csv", 7)
    gt = _load_csv(seq / "mav0/state_groundtruth_estimate0/data.csv", 17)
    t_bs_gt = _load_t_bs(seq / "mav0/state_groundtruth_estimate0/sensor.yaml")
    t_bs_imu = _load_t_bs(seq / "mav0/imu0/sensor.yaml")

    t_gt = gt[:, 0] * 1e-9
    t_imu = imu[:, 0] * 1e-9
    t_rel = t_gt - t_gt[0]
    if static_window_s is None:
        static_window_s = _stationary_window(t_rel, gt[:, 8:11], duration_s)
    start_s, end_s = static_window_s
    mask = (t_rel >= start_s) & (t_rel <= end_s)
    if np.count_nonzero(mask) < 20:
        raise ValueError("selected stationary window contains fewer than 20 GT samples")

    selected_t = t_gt[mask]
    velocity_world = gt[mask, 8:11]
    acceleration_world = acceleration_from_groundtruth(velocity_world, selected_t)

    # ASL gives T_R_Sgt. Convert it to T_R_Simu using
    # T_R_Simu = T_R_Sgt * inv(T_B_Sgt) * T_B_Simu.
    rotation_gt_to_world = _quat_wxyz_to_rotation(gt[mask, 4:8])
    rotation_gt_to_imu = np.linalg.inv(t_bs_gt[:3, :3]) @ t_bs_imu[:3, :3]
    rotation_imu_to_world = rotation_gt_to_world @ rotation_gt_to_imu
    bias_gt = gt[mask, 14:17]
    bias_imu = np.einsum("ij,nj->ni", rotation_gt_to_imu.T, bias_gt)
    predicted = predicted_specific_force(acceleration_world, rotation_imu_to_world, bias_imu)

    measured = np.stack(
        [np.interp(selected_t, t_imu, imu[:, 4 + axis]) for axis in range(3)], axis=1
    )
    residual = measured - predicted
    axis_mae = np.mean(np.abs(residual), axis=0)
    mean_vector_error = float(np.mean(np.linalg.norm(residual, axis=1)))
    rmse = float(np.sqrt(np.mean(residual * residual)))
    mean_speed = float(np.mean(np.linalg.norm(velocity_world, axis=1)))
    passed = mean_speed <= max_static_speed_mps and mean_vector_error <= tol_mps2

    if plot_path is not None:
        import matplotlib.pyplot as plt

        plot = Path(plot_path)
        plot.parent.mkdir(parents=True, exist_ok=True)
        figure, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        relative_time = selected_t - selected_t[0]
        labels = ("x", "y", "z")
        for axis, label in enumerate(labels):
            axes[axis].plot(relative_time, measured[:, axis], label="measured", linewidth=1)
            axes[axis].plot(relative_time, predicted[:, axis], label="GT predicted", linewidth=1)
            axes[axis].set_ylabel(f"a_{label} [m/s²]")
            axes[axis].grid(alpha=0.25)
        axes[0].legend()
        axes[-1].set_xlabel("time in stationary window [s]")
        figure.tight_layout()
        figure.savefig(plot, dpi=160)
        plt.close(figure)

    return Gate1Result(
        passed=passed,
        start_s=start_s,
        end_s=end_s,
        mean_speed_mps=mean_speed,
        mean_error_mps2=mean_vector_error,
        rmse_mps2=rmse,
        axis_mae_mps2=axis_mae,
        samples=int(np.count_nonzero(mask)),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("seq_dir", help="EuRoC sequence directory containing mav0/")
    parser.add_argument("--window", nargs=2, type=float, metavar=("START_S", "END_S"),
                        help="stationary window relative to the first GT timestamp")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="automatic stationary-window duration (default: 5 seconds)")
    parser.add_argument("--tolerance", type=float, default=0.3,
                        help="maximum mean vector discrepancy in m/s²")
    parser.add_argument("--max-static-speed", type=float, default=0.05,
                        help="maximum mean GT speed in the selected window")
    parser.add_argument("--plot", help="optional output plot path")
    args = parser.parse_args()

    window = tuple(args.window) if args.window else None
    result = check_gate1(
        args.seq_dir,
        static_window_s=window,
        tol_mps2=args.tolerance,
        duration_s=args.duration,
        max_static_speed_mps=args.max_static_speed,
        plot_path=args.plot,
    )
    status = "PASS" if result.passed else "FAIL"
    print(f"[gate1] {status}")
    print(f"[gate1] stationary_window_s={result.start_s:.3f}..{result.end_s:.3f} "
          f"samples={result.samples} mean_speed_mps={result.mean_speed_mps:.6f}")
    print(f"[gate1] mean_vector_error_mps2={result.mean_error_mps2:.6f} "
          f"rmse_mps2={result.rmse_mps2:.6f} "
          f"axis_mae_mps2={result.axis_mae_mps2.tolist()}")
    raise SystemExit(0 if result.passed else 1)


if __name__ == "__main__":
    main()
