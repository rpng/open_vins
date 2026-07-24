#!/usr/bin/env python3
"""Validate the structural and numerical integrity of a Stage-1 HDF5 dump."""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np


def validate(
    path: Path,
    min_frames: int = 100,
    min_gt_reprojection_fraction: float = 0.8,
    require_mixed_gate: bool = True,
) -> dict[str, float | int | str]:
    required = {
        "/frames/timestamp": (None,),
        "/frames/state": (None, 16),
        "/frames/groundtruth": (None, 17),
        "/frames/covariance_imu15": (None, 15, 15),
        "/frames/imu_window": (None, 20, 6),
        "/frames/diagnostics": (None, 5),
        "/features/diagnostics": (None, 12),
    }
    with h5py.File(path, "r") as dump:
        for name, expected in required.items():
            if name not in dump:
                raise ValueError(f"missing dataset: {name}")
            shape = dump[name].shape
            if len(shape) != len(expected) or any(
                want is not None and got != want for got, want in zip(shape, expected)
            ):
                raise ValueError(f"{name}: shape {shape}, expected {expected}")

        timestamps = dump["/frames/timestamp"][:]
        frame_count = len(timestamps)
        if frame_count < min_frames:
            raise ValueError(f"only {frame_count} frames; expected at least {min_frames}")
        for name in required:
            if name.startswith("/frames/") and dump[name].shape[0] != frame_count:
                raise ValueError(f"{name}: row count does not match timestamps")
        if not np.all(np.isfinite(timestamps)) or not np.all(np.diff(timestamps) > 0):
            raise ValueError("frame timestamps are non-finite or not strictly increasing")

        state = dump["/frames/state"][:]
        groundtruth = dump["/frames/groundtruth"][:]
        covariance = dump["/frames/covariance_imu15"][:]
        windows = dump["/frames/imu_window"][:]
        features = dump["/features/diagnostics"][:]
        if not all(np.all(np.isfinite(array)) for array in (state, groundtruth, covariance, windows)):
            raise ValueError("a required frame dataset contains non-finite values")
        symmetry_error = float(np.max(np.abs(covariance - covariance.transpose(0, 2, 1))))
        if symmetry_error > 1e-8:
            raise ValueError(f"IMU covariance is not symmetric (max error {symmetry_error})")
        if np.any(np.diagonal(covariance, axis1=1, axis2=2) <= 0):
            raise ValueError("IMU covariance contains a non-positive diagonal entry")
        if features.shape[0] == 0:
            raise ValueError("no pre-gate feature diagnostics were recorded")
        if not np.all(np.isfinite(features[:, [0, 1, 2, 6, 8, 9, 10, 11]])):
            raise ValueError("required feature diagnostic columns contain non-finite values")
        pass_rate = float(np.mean(features[:, 11]))
        if not 0.0 <= pass_rate <= 1.0:
            raise ValueError(f"feature gate pass rate {pass_rate:.3f} is outside [0,1]")
        if require_mixed_gate and not 0.0 < pass_rate < 1.0:
            raise ValueError(f"feature gate pass rate {pass_rate:.3f} does not contain both outcomes")
        gt_reprojection_fraction = float(np.mean(np.isfinite(features[:, 7])))
        if gt_reprojection_fraction < min_gt_reprojection_fraction:
            raise ValueError(
                "finite GT reprojection fraction "
                f"{gt_reprojection_fraction:.3f} is below {min_gt_reprojection_fraction:.3f}; "
                "check the estimator-to-groundtruth frame alignment"
            )

        sequence = dump["/meta"].attrs.get("sequence", "")
        if isinstance(sequence, bytes):
            sequence = sequence.decode()
        return {
            "sequence": str(sequence),
            "frames": frame_count,
            "features": int(features.shape[0]),
            "gate_pass_rate": pass_rate,
            "covariance_symmetry_max": symmetry_error,
            "duration_s": float(timestamps[-1] - timestamps[0]),
            "gt_reprojection_finite_fraction": gt_reprojection_fraction,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", type=Path)
    parser.add_argument("--min-frames", type=int, default=100)
    parser.add_argument("--min-gt-reprojection-fraction", type=float, default=0.8)
    parser.add_argument(
        "--allow-degenerate-gate",
        action="store_true",
        help="permit all-pass/all-reject learned arms while still reporting the rate",
    )
    args = parser.parse_args()
    summary = validate(
        args.dump,
        args.min_frames,
        args.min_gt_reprojection_fraction,
        require_mixed_gate=not args.allow_degenerate_gate,
    )
    print("[stage1-validate] PASS")
    for key, value in summary.items():
        print(f"[stage1-validate] {key}={value}")


if __name__ == "__main__":
    main()
