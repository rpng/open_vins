#!/usr/bin/env python3
"""
hdf5_dump_dataset.py  --  loads the Stage-1 HDF5 dumps for Stage-2 training.

Companion to: Section 8.3 (offline training) and Section 18.3 (what DiagnosticsLogger writes).

This is the bridge between the C++ Stage 1 and the Python Stage 2. It reads the per-sequence
HDF5 files produced by conformal/stage1_dumps/run_asl_msckf.cpp and yields:
    * for Net A: variable-K per-feature tensors + frame context + per-feature error targets
    * for Net B: [20, 6] IMU windows + preintegration-error targets

CRITICAL -- SEQUENCE-DISJOINT SPLITS (Section 9.1 / risk table): calibration and test must
NEVER share a trajectory, or coverage leaks and the method appears to work when it does not.
This loader therefore splits by SEQUENCE, not by frame. Frame-level splitting is exactly the
failure ablation A7 quantifies (conformal/experiments/ablation_a7_split_leakage.py) -- do not use
it for real training/calibration. Because whole sequences are consumed per split, the number
of SEQUENCES (EuRoC has only 11) is the binding resource constraint of the whole project.

Net B uses sidecars made by derive_netb_targets.py.  Keeping derived supervision separate
preserves the immutable Stage-1 artifacts and records exactly how the four targets were made.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset

try:
    import h5py
except ImportError:  # NPZ exports allow use in lean PyTorch containers.
    h5py = None

IMAGE_WIDTH = 752.0
IMAGE_HEIGHT = 480.0
MAX_VALID_REPROJECTION_ERROR = float(np.hypot(IMAGE_WIDTH, IMAGE_HEIGHT))


@dataclass
class Split:
    """A sequence-disjoint split. Each field is a list of sequence names (not frames)."""
    train: list[str]
    calibration: list[str]  # held-out pool for conformal q_alpha (Section 9)
    test: list[str]


# Example EuRoC split. TODO(intern): finalise; keep calibration+test disjoint from train.
EUROC_SEQUENCE_DISJOINT_SPLIT = Split(
    train=["MH_01_easy", "MH_02_easy", "MH_03_medium", "V1_01_easy", "V1_02_medium", "V2_01_easy"],
    calibration=["MH_04_difficult", "V1_03_difficult"],
    test=["MH_05_difficult", "V2_02_medium", "V2_03_difficult"],
)


class NetADump(Dataset):
    """Per-frame visual samples for Net A: (features[K,D], mask[K], frame_ctx[C], err[K])."""

    def __init__(self, h5_dir: str | Path, sequences: list[str]) -> None:
        self.h5_dir = Path(h5_dir)
        self.sequences = sequences
        self.samples: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
        for sequence in sequences:
            npz_path = self.h5_dir / f"{sequence}_stage2_neta.npz"
            if npz_path.exists():
                with np.load(npz_path) as archive:
                    features = archive["features"]
                    contexts = archive["frame_context"]
                    errors = archive["errors"]
                    offsets = archive["frame_offsets"]
                    for frame_index, context in enumerate(contexts):
                        start, stop = int(offsets[frame_index]), int(offsets[frame_index + 1])
                        self.samples.append((
                            features[start:stop].copy(), context.copy(), errors[start:stop].copy()
                        ))
                continue
            if h5py is None:
                raise ImportError(
                    f"h5py is unavailable and portable export {npz_path} does not exist"
                )
            path = self.h5_dir / f"{sequence}_stage1.h5"
            with h5py.File(path, "r") as dump:
                frame_t = dump["/frames/timestamp"][:]
                frame_diag = dump["/frames/diagnostics"][:]
                feature_diag = dump["/features/diagnostics"][:]
            if feature_diag.size == 0:
                continue
            frame_groups: list[list[int]] = [[] for _ in range(len(frame_t))]
            nearest = np.searchsorted(frame_t, feature_diag[:, 0])
            nearest = np.clip(nearest, 0, len(frame_t) - 1)
            use_previous = (
                (nearest > 0)
                & (np.abs(feature_diag[:, 0] - frame_t[nearest - 1])
                   < np.abs(feature_diag[:, 0] - frame_t[nearest]))
            )
            nearest[use_previous] -= 1
            for row, frame_index in enumerate(nearest):
                if abs(float(feature_diag[row, 0] - frame_t[frame_index])) <= 1e-5:
                    frame_groups[int(frame_index)].append(row)

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
                # Eight inference-time diagnostics available in schema v1.  Bounded/log
                # transforms prevent pixel coordinates and chi2 from dominating.
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
                errors = table[:, 7].astype(np.float32)
                diag = frame_diag[frame_index]
                context = np.asarray((
                    np.log1p(max(diag[1], 0.0)),
                    np.log1p(max(diag[2], 0.0)),
                    diag[3] / 255.0,
                    diag[4] / 1000.0,
                    np.log1p(len(table)),
                    float(np.mean(table[:, 11] > 0.5)),
                ), dtype=np.float32)
                if np.isfinite(features).all() and np.isfinite(context).all():
                    self.samples.append((features, context, errors))

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, i: int):
        features, context, errors = self.samples[i]
        return torch.from_numpy(features), torch.from_numpy(context), torch.from_numpy(errors)


class NetBDump(Dataset):
    """Per-frame inertial samples for Net B: (imu_window[20,6], preint_error[scalar-or-4])."""

    def __init__(self, h5_dir: str | Path, sequences: list[str],
                 sidecar_dir: str | Path | None = None) -> None:
        self.h5_dir = Path(h5_dir)
        self.sidecar_dir = Path(sidecar_dir) if sidecar_dir is not None else self.h5_dir
        self.sequences = sequences
        self.windows: list[np.ndarray] = []
        self.targets: list[np.ndarray] = []
        for sequence in sequences:
            npz_path = self.sidecar_dir / f"{sequence}_stage2_netb.npz"
            if npz_path.exists():
                with np.load(npz_path) as archive:
                    windows = archive["imu_window"].astype(np.float32)
                    targets = archive["target_normalized"].astype(np.float32)
                if not np.isfinite(windows).all() or not np.isfinite(targets).all():
                    raise ValueError(f"{npz_path}: arrays contain non-finite values")
                self.windows.extend(windows)
                self.targets.extend(targets)
                continue
            if h5py is None:
                raise ImportError(
                    f"h5py is unavailable and portable export {npz_path} does not exist"
                )
            path = self.sidecar_dir / f"{sequence}_stage2_netb.h5"
            with h5py.File(path, "r") as sidecar:
                valid = sidecar["/valid"][:].astype(bool)
                windows = sidecar["/imu_window"][:][valid].astype(np.float32)
                targets = sidecar["/target_normalized"][:][valid].astype(np.float32)
            if not np.isfinite(windows).all() or not np.isfinite(targets).all():
                raise ValueError(f"{path}: valid rows contain non-finite values")
            self.windows.extend(windows)
            self.targets.extend(targets)

    def __len__(self) -> int:
        return len(self.windows)

    def __getitem__(self, i: int):
        return torch.from_numpy(self.windows[i]), torch.from_numpy(self.targets[i])


def collate_variable_k(batch):
    """Pad a batch of variable-K Net A frames to [B, K_max, D] + mask [B, K_max]."""
    if not batch:
        raise ValueError("cannot collate an empty batch")
    batch_size = len(batch)
    max_features = max(features.shape[0] for features, _, _ in batch)
    feature_dim = batch[0][0].shape[1]
    context_dim = batch[0][1].shape[0]
    features_out = torch.zeros((batch_size, max_features, feature_dim), dtype=torch.float32)
    mask_out = torch.zeros((batch_size, max_features), dtype=torch.float32)
    context_out = torch.zeros((batch_size, context_dim), dtype=torch.float32)
    errors_out = torch.zeros((batch_size, max_features), dtype=torch.float32)
    for row, (features, context, errors) in enumerate(batch):
        count = features.shape[0]
        features_out[row, :count] = features
        mask_out[row, :count] = 1.0
        context_out[row] = context
        errors_out[row, :count] = errors
    return features_out, mask_out, context_out, errors_out
