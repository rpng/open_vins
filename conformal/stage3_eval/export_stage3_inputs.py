#!/usr/bin/env python3
"""Export inference-only Stage-3 inputs from held-out Stage-1 HDF5 dumps.

Every row with finite inference diagnostics is retained regardless of its GT
error. Ground-truth arrays are carried separately and are read only by the
oracle arm.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np


TEST_SEQUENCES = ("MH_05_difficult", "V2_02_medium", "V2_03_difficult")
IMAGE_WIDTH = 752.0
IMAGE_HEIGHT = 480.0


def export_sequence(stage1_path: Path, netb_path: Path, output_path: Path) -> dict[str, int]:
    with h5py.File(stage1_path, "r") as dump:
        frame_t = dump["/frames/timestamp"][:].astype(np.float64)
        frame_diag = dump["/frames/diagnostics"][:].astype(np.float64)
        imu_window = dump["/frames/imu_window"][:].astype(np.float32)
        feature_diag = dump["/features/diagnostics"][:].astype(np.float64)
    with h5py.File(netb_path, "r") as netb:
        if not np.array_equal(netb["/frame_timestamp"][:], frame_t):
            raise ValueError(f"{netb_path}: frame timestamps differ from Stage 1")
        inertial_oracle = netb["/target_physical_density"][:].astype(np.float64)
        inertial_oracle_valid = netb["/valid"][:].astype(np.uint8)

    groups: list[list[int]] = [[] for _ in range(len(frame_t))]
    nearest = np.clip(np.searchsorted(frame_t, feature_diag[:, 0]), 0, len(frame_t) - 1)
    use_previous = (
        (nearest > 0)
        & (np.abs(feature_diag[:, 0] - frame_t[nearest - 1])
           < np.abs(feature_diag[:, 0] - frame_t[nearest]))
    )
    nearest[use_previous] -= 1
    for row, frame_index in enumerate(nearest):
        if abs(float(feature_diag[row, 0] - frame_t[frame_index])) <= 1e-5:
            groups[int(frame_index)].append(row)

    feature_blocks = []
    feature_t_blocks = []
    feature_id_blocks = []
    gt_error_blocks = []
    contexts = []
    visual_frame_t = []
    offsets = [0]
    for frame_index, rows in enumerate(groups):
        if not rows:
            continue
        table = feature_diag[rows]
        inference_columns = table[:, [2, 3, 4, 5, 6, 8, 9, 10]]
        valid = np.isfinite(inference_columns).all(axis=1)
        table = table[valid]
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
        if not np.isfinite(context).all():
            continue
        feature_blocks.append(features)
        feature_t_blocks.append(table[:, 0].astype(np.float64))
        feature_id_blocks.append(table[:, 1].astype(np.uint64))
        gt_error_blocks.append(table[:, 7].astype(np.float64))
        contexts.append(context)
        visual_frame_t.append(frame_t[frame_index])
        offsets.append(offsets[-1] + len(table))

    features = np.concatenate(feature_blocks) if feature_blocks else np.empty((0, 8), np.float32)
    feature_t = np.concatenate(feature_t_blocks) if feature_t_blocks else np.empty(0, np.float64)
    feature_id = np.concatenate(feature_id_blocks) if feature_id_blocks else np.empty(0, np.uint64)
    gt_error = np.concatenate(gt_error_blocks) if gt_error_blocks else np.empty(0, np.float64)
    np.savez_compressed(
        output_path,
        frame_timestamp=frame_t,
        imu_window=imu_window,
        inertial_oracle_density=inertial_oracle,
        inertial_oracle_valid=inertial_oracle_valid,
        visual_frame_timestamp=np.asarray(visual_frame_t, dtype=np.float64),
        visual_frame_offsets=np.asarray(offsets, dtype=np.int64),
        visual_frame_context=np.asarray(contexts, dtype=np.float32),
        visual_features=features,
        feature_timestamp=feature_t,
        feature_id=feature_id,
        gt_visual_error=gt_error,
    )
    return {
        "frames": len(frame_t),
        "visual_frames": len(contexts),
        "features": len(features),
        "finite_gt_visual": int(np.isfinite(gt_error).sum()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage1-dir", type=Path, required=True)
    parser.add_argument(
        "--input-pattern", default="{sequence}_stage1.h5",
        help="filename pattern below --stage1-dir; use {sequence}_stock.h5 for pilot runs",
    )
    parser.add_argument("--netb-target-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sequences", nargs="+", default=list(TEST_SEQUENCES))
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for sequence in args.sequences:
        summary = export_sequence(
            args.stage1_dir / args.input_pattern.format(sequence=sequence),
            args.netb_target_dir / f"{sequence}_stage2_netb.h5",
            args.out_dir / f"{sequence}_stage3_inputs.npz",
        )
        print(sequence, summary)


if __name__ == "__main__":
    main()
