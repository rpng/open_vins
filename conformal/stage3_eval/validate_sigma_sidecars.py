#!/usr/bin/env python3
"""Validate every Stage-3 HDF5 sigma sidecar before filter execution."""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sidecar-dir", type=Path, required=True)
    parser.add_argument("--expected", type=int, default=12)
    args = parser.parse_args()
    paths = sorted(args.sidecar_dir.glob("*_sigma.h5"))
    if len(paths) != args.expected:
        raise ValueError(f"expected {args.expected} sidecars, found {len(paths)}")
    for path in paths:
        with h5py.File(path, "r") as data:
            frame_t = data["/frames/timestamp"][:]
            imu = data["/frames/imu_sigmas"][:]
            feature_t = data["/features/timestamp"][:]
            feature_id = data["/features/feature_id"][:]
            visual = data["/features/sigma_pix"][:]
            if imu.shape != (len(frame_t), 4):
                raise ValueError(f"{path}: invalid IMU shape {imu.shape}")
            if not (len(feature_t) == len(feature_id) == len(visual)):
                raise ValueError(f"{path}: inconsistent feature lengths")
            if np.any(np.diff(frame_t) <= 0):
                raise ValueError(f"{path}: non-monotonic frame timestamps")
            if not np.isfinite(imu).all() or np.any(imu <= 0):
                raise ValueError(f"{path}: invalid IMU sigma")
            if not np.isfinite(visual).all() or np.any(visual <= 0):
                raise ValueError(f"{path}: invalid visual sigma")
            keys = np.rec.fromarrays((np.rint(feature_t * 1e6).astype(np.int64), feature_id))
            if len(np.unique(keys)) != len(keys):
                raise ValueError(f"{path}: duplicate feature timestamp/id keys")
        print(path.name, "frames", len(frame_t), "features", len(visual), "PASS")


if __name__ == "__main__":
    main()
