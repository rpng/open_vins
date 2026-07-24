#!/usr/bin/env python3
"""Export compact Stage-2 training arrays from HDF5 for PyTorch-only images.

The remote PyTorch container need not contain h5py.  This exporter runs in the
validated Stage-1 image and writes one portable NPZ for each head and sequence.
Net-A variable-length frames use flat arrays plus CSR-style frame offsets.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np

from derive_netb_targets import SEQUENCES

IMAGE_WIDTH = 752.0
IMAGE_HEIGHT = 480.0
MAX_VALID_REPROJECTION_ERROR = float(np.hypot(IMAGE_WIDTH, IMAGE_HEIGHT))


def export_neta(source_path: Path, destination: Path) -> tuple[int, int]:
    with h5py.File(source_path, "r") as dump:
        frame_t = dump["/frames/timestamp"][:]
        frame_diag = dump["/frames/diagnostics"][:]
        feature_diag = dump["/features/diagnostics"][:]
    frame_groups: list[list[int]] = [[] for _ in range(len(frame_t))]
    if len(feature_diag):
        nearest = np.clip(np.searchsorted(frame_t, feature_diag[:, 0]), 0, len(frame_t) - 1)
        use_previous = (
            (nearest > 0)
            & (np.abs(feature_diag[:, 0] - frame_t[nearest - 1])
               < np.abs(feature_diag[:, 0] - frame_t[nearest]))
        )
        nearest[use_previous] -= 1
        for row, frame_index in enumerate(nearest):
            if abs(float(feature_diag[row, 0] - frame_t[frame_index])) <= 1e-5:
                frame_groups[int(frame_index)].append(row)

    feature_blocks = []
    error_blocks = []
    contexts = []
    timestamps = []
    offsets = [0]
    for frame_index, rows in enumerate(frame_groups):
        if not rows:
            continue
        table = feature_diag[rows]
        valid_target = (
            np.isfinite(table[:, 7])
            & (table[:, 7] >= 0.0)
            & (table[:, 7] <= MAX_VALID_REPROJECTION_ERROR)
        )
        table = table[valid_target]
        if len(table) == 0:
            continue
        features = np.column_stack((
            np.log1p(np.maximum(table[:, 2], 0.0)),
            np.clip(table[:, 3], 0.0, 1.0),
            table[:, 4] / IMAGE_WIDTH,
            table[:, 5] / IMAGE_HEIGHT,
            np.log1p(np.maximum(table[:, 6], 0.0)),
            np.log1p(np.maximum(table[:, 8], 0.0)),
            np.log1p(np.maximum(table[:, 9], 0.0)),
            table[:, 10],
        )).astype(np.float32)
        diag = frame_diag[frame_index]
        context = np.asarray((
            np.log1p(max(diag[1], 0.0)),
            np.log1p(max(diag[2], 0.0)),
            diag[3] / 255.0,
            diag[4] / 1000.0,
            np.log1p(len(table)),
            float(np.mean(table[:, 11] > 0.5)),
        ), dtype=np.float32)
        if not (np.isfinite(features).all() and np.isfinite(context).all()):
            continue
        feature_blocks.append(features)
        error_blocks.append(table[:, 7].astype(np.float32))
        contexts.append(context)
        timestamps.append(frame_t[frame_index])
        offsets.append(offsets[-1] + len(table))

    flat_features = np.concatenate(feature_blocks, axis=0) if feature_blocks else np.empty((0, 8), np.float32)
    flat_errors = np.concatenate(error_blocks) if error_blocks else np.empty(0, np.float32)
    np.savez_compressed(
        destination,
        features=flat_features,
        errors=flat_errors,
        frame_context=np.asarray(contexts, dtype=np.float32),
        frame_timestamp=np.asarray(timestamps, dtype=np.float64),
        frame_offsets=np.asarray(offsets, dtype=np.int64),
        max_valid_reprojection_error_px=np.asarray(MAX_VALID_REPROJECTION_ERROR),
    )
    return len(contexts), len(flat_errors)


def export_netb(source_path: Path, destination: Path) -> int:
    with h5py.File(source_path, "r") as sidecar:
        valid = sidecar["/valid"][:].astype(bool)
        np.savez_compressed(
            destination,
            imu_window=sidecar["/imu_window"][:][valid].astype(np.float32),
            target_normalized=sidecar["/target_normalized"][:][valid].astype(np.float32),
            target_physical_density=sidecar["/target_physical_density"][:][valid],
            frame_timestamp=sidecar["/frame_timestamp"][:][valid],
            interval_dt=sidecar["/interval_dt"][:][valid],
        )
    return int(valid.sum())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage1-dir", type=Path, required=True)
    parser.add_argument("--netb-sidecar-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sequences", nargs="+", default=list(SEQUENCES))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for sequence in args.sequences:
        neta_frames, neta_features = export_neta(
            args.stage1_dir / f"{sequence}_stage1.h5",
            args.out_dir / f"{sequence}_stage2_neta.npz",
        )
        netb_frames = export_netb(
            args.netb_sidecar_dir / f"{sequence}_stage2_netb.h5",
            args.out_dir / f"{sequence}_stage2_netb.npz",
        )
        print(
            f"{sequence}: NetA frames={neta_frames} features={neta_features}; "
            f"NetB intervals={netb_frames}"
        )


if __name__ == "__main__":
    main()
