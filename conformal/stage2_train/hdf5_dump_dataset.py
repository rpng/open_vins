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

TODO(intern): implement the h5 reads (keys mirror DiagnosticsLogger.hpp's groups). Padding of
variable-K feature sets into a [B, K_max, D] tensor + a [B, K_max] mask goes in the collate fn.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch
from torch.utils.data import Dataset


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
        self.index: list[tuple[str, int]] = []  # (sequence, frame_idx)
        # TODO(intern): open each <seq>.h5, enumerate /features/frame_XXXXX, fill self.index.

    def __len__(self) -> int:
        return len(self.index)

    def __getitem__(self, i: int):
        raise NotImplementedError("TODO(intern): read one frame's per-feature table + targets from HDF5")


class NetBDump(Dataset):
    """Per-frame inertial samples for Net B: (imu_window[20,6], preint_error[scalar-or-4])."""

    def __init__(self, h5_dir: str | Path, sequences: list[str]) -> None:
        self.h5_dir = Path(h5_dir)
        self.sequences = sequences
        self.index: list[tuple[str, int]] = []
        # TODO(intern): enumerate /imu_windows across the given sequences.

    def __len__(self) -> int:
        return len(self.index)

    def __getitem__(self, i: int):
        raise NotImplementedError("TODO(intern): read one [20,6] window + preintegration-error target")


def collate_variable_k(batch):
    """Pad a batch of variable-K Net A frames to [B, K_max, D] + mask [B, K_max]."""
    raise NotImplementedError("TODO(intern): pad per-feature sets and build the mask")
