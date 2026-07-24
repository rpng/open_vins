#!/usr/bin/env python3
"""Convert portable Net-A arrays into the self-verifying C++ HDF5 format."""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np


WEIGHTS = (
    "encoder0_weight", "encoder0_bias",
    "encoder2_weight", "encoder2_bias",
    "pool_weight", "pool_bias",
    "head0_weight", "head0_bias",
    "head2_weight", "head2_bias",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with np.load(args.input) as archive, h5py.File(args.output, "w") as output:
        weights = output.create_group("weights")
        for name in WEIGHTS:
            values = archive[name].astype(np.float64)
            if not np.isfinite(values).all():
                raise ValueError(f"{name} contains non-finite values")
            weights.create_dataset(name, data=values)
        verification = output.create_group("verification")
        verification.create_dataset(
            "features", data=archive["verification_features"].astype(np.float64)
        )
        verification.create_dataset(
            "frame_context",
            data=archive["verification_frame_context"].astype(np.float64),
        )
        verification.create_dataset(
            "log_sigma", data=archive["verification_log_sigma"].astype(np.float64)
        )
        output.attrs["format"] = "openvins-live-neta-v1"
        output.attrs["inference"] = "causal batch DeepSets"
    print(f"[neta-h5] PASS output={args.output}")


if __name__ == "__main__":
    main()
