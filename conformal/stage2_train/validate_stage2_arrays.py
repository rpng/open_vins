#!/usr/bin/env python3
"""Validate portable Stage-2 arrays and summarize target distributions."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


SPLITS = {
    "train": ["MH_01_easy", "MH_02_easy", "MH_03_medium", "V1_01_easy", "V1_02_medium", "V2_01_easy"],
    "calibration": ["MH_04_difficult", "V1_03_difficult"],
    "test": ["MH_05_difficult", "V2_02_medium", "V2_03_difficult"],
}
CHANNELS = ("sigma_w", "sigma_a", "sigma_wb", "sigma_ab")
MAX_VALID_REPROJECTION_ERROR = float(np.hypot(752.0, 480.0))


def _split_for(sequence: str) -> str:
    matches = [name for name, members in SPLITS.items() if sequence in members]
    if len(matches) != 1:
        raise ValueError(f"{sequence}: expected exactly one split membership, got {matches}")
    return matches[0]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--array-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    expected = {sequence for members in SPLITS.values() for sequence in members}
    seen: set[str] = set()
    summaries: list[dict[str, object]] = []
    failures: list[str] = []
    split_targets: dict[str, list[np.ndarray]] = {name: [] for name in SPLITS}

    for sequence in sorted(expected):
        neta_path = args.array_dir / f"{sequence}_stage2_neta.npz"
        netb_path = args.array_dir / f"{sequence}_stage2_netb.npz"
        if not neta_path.exists() or not netb_path.exists():
            failures.append(f"{sequence}: missing Net-A or Net-B NPZ")
            continue
        seen.add(sequence)
        with np.load(neta_path) as neta:
            features = neta["features"]
            errors = neta["errors"]
            context = neta["frame_context"]
            offsets = neta["frame_offsets"]
            neta_t = neta["frame_timestamp"]
        with np.load(netb_path) as netb:
            windows = netb["imu_window"]
            normalized = netb["target_normalized"]
            physical = netb["target_physical_density"]
            netb_t = netb["frame_timestamp"]
            interval_dt = netb["interval_dt"]

        if features.ndim != 2 or features.shape[1] != 8:
            failures.append(f"{sequence}: Net-A feature shape {features.shape}")
        if context.ndim != 2 or context.shape[1] != 6:
            failures.append(f"{sequence}: Net-A context shape {context.shape}")
        if len(errors) != len(features) or offsets[-1] != len(features):
            failures.append(f"{sequence}: inconsistent Net-A flat arrays")
        if len(errors) and (errors.min() < 0.0 or errors.max() > MAX_VALID_REPROJECTION_ERROR):
            failures.append(
                f"{sequence}: Net-A target outside geometric range "
                f"[0,{MAX_VALID_REPROJECTION_ERROR}]"
            )
        if len(offsets) != len(context) + 1:
            failures.append(f"{sequence}: inconsistent Net-A offsets")
        if windows.ndim != 3 or windows.shape[1:] != (20, 6):
            failures.append(f"{sequence}: Net-B window shape {windows.shape}")
        if normalized.shape != (len(windows), 4) or physical.shape != normalized.shape:
            failures.append(f"{sequence}: Net-B target shape mismatch")
        arrays = (features, errors, context, windows, normalized, physical, interval_dt)
        if any(not np.isfinite(array).all() for array in arrays):
            failures.append(f"{sequence}: non-finite portable arrays")
        if np.any(np.diff(neta_t) <= 0) or np.any(np.diff(netb_t) <= 0):
            failures.append(f"{sequence}: non-monotonic timestamps")
        if np.any((interval_dt <= 0) | (interval_dt > 0.25)):
            failures.append(f"{sequence}: invalid target interval")
        for channel, name in enumerate(CHANNELS):
            if np.std(normalized[:, channel]) < 1e-8:
                failures.append(f"{sequence}: degenerate {name} target")

        split = _split_for(sequence)
        split_targets[split].append(normalized)
        row: dict[str, object] = {
            "sequence": sequence,
            "split": split,
            "neta_frames": len(context),
            "neta_features": len(features),
            "neta_error_median_px": float(np.median(errors)),
            "neta_error_p95_px": float(np.quantile(errors, 0.95)),
            "netb_intervals": len(windows),
            "netb_interval_median_s": float(np.median(interval_dt)),
        }
        for channel, name in enumerate(CHANNELS):
            row[f"{name}_physical_median"] = float(np.median(physical[:, channel]))
            row[f"{name}_normalized_median"] = float(np.median(normalized[:, channel]))
            row[f"{name}_normalized_p95"] = float(np.quantile(normalized[:, channel], 0.95))
            row[f"{name}_zero_fraction"] = float(np.mean(normalized[:, channel] == 0.0))
        summaries.append(row)

    for split, blocks in split_targets.items():
        if not blocks:
            failures.append(f"{split}: no target arrays")
            continue
        target = np.concatenate(blocks)
        for channel, name in enumerate(CHANNELS):
            if float(np.mean(target[:, channel] == 0.0)) > 0.95:
                failures.append(f"{split}: {name} is more than 95% exactly zero")

    csv_path = args.out_dir / "stage2_data_summary.csv"
    if summaries:
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
    report = {
        "status": "PASS" if not failures and seen == expected else "FAIL",
        "sequences_expected": len(expected),
        "sequences_seen": len(seen),
        "splits": SPLITS,
        "failures": failures,
    }
    report_path = args.out_dir / "stage2_data_validation.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
