#!/usr/bin/env python3
"""Convert portable Stage-3 sigma predictions into the runner HDF5 schema."""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for source in sorted(args.input_dir.glob("*_sigma.npz")):
        with np.load(source) as data:
            arm = str(data["arm"])
            sequence = str(data["sequence"])
            output = args.out_dir / source.with_suffix(".h5").name
            with h5py.File(output, "w") as target:
                meta = target.create_group("meta")
                meta.attrs["schema_version"] = "1"
                meta.attrs["arm"] = arm
                meta.attrs["sequence"] = sequence
                frames = target.create_group("frames")
                frames.create_dataset("timestamp", data=data["frame_timestamp"])
                frames.create_dataset("imu_sigmas", data=data["imu_sigmas"])
                features = target.create_group("features")
                features.create_dataset("timestamp", data=data["feature_timestamp"])
                features.create_dataset("feature_id", data=data["feature_id"])
                features.create_dataset("sigma_pix", data=data["visual_sigmas"])
        print(output)


if __name__ == "__main__":
    main()
