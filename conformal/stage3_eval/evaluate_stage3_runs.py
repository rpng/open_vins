#!/usr/bin/env python3
"""Compute aligned trajectory metrics and diagnostic summaries for Stage 3."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import h5py
import numpy as np


TEST_SEQUENCES = ("MH_05_difficult", "V2_02_medium", "V2_03_difficult")
ALL_SEQUENCES = (
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult",
    "MH_05_difficult", "V1_01_easy", "V1_02_medium", "V1_03_difficult",
    "V2_01_easy", "V2_02_medium", "V2_03_difficult",
)
ARMS = ("stock", "learned", "conformalised", "oracle")
SPLIT_BY_SEQUENCE = {
    **{name: "train" for name in (
        "MH_01_easy", "MH_02_easy", "MH_03_medium",
        "V1_01_easy", "V1_02_medium", "V2_01_easy",
    )},
    **{name: "calibration" for name in ("MH_04_difficult", "V1_03_difficult")},
    **{name: "test" for name in TEST_SEQUENCES},
}


def _rigid_align(estimate: np.ndarray, truth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mean_e = estimate.mean(axis=0)
    mean_t = truth.mean(axis=0)
    covariance = (estimate - mean_e).T @ (truth - mean_t)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = mean_t - rotation @ mean_e
    return rotation, translation


def _metrics(path: Path) -> dict[str, float | int]:
    with h5py.File(path, "r") as data:
        timestamps = data["/frames/timestamp"][:]
        estimate = data["/frames/state"][:, 4:7]
        truth = data["/frames/groundtruth"][:, 5:8]
        features = data["/features/diagnostics"][:]
    valid = np.isfinite(estimate).all(axis=1) & np.isfinite(truth).all(axis=1)
    timestamps, estimate, truth = timestamps[valid], estimate[valid], truth[valid]
    rotation, translation = _rigid_align(estimate, truth)
    aligned = (rotation @ estimate.T).T + translation
    ate = np.linalg.norm(aligned - truth, axis=1)

    relative = []
    for i, timestamp in enumerate(timestamps):
        j = int(np.searchsorted(timestamps, timestamp + 1.0))
        if j < len(timestamps) and abs(float(timestamps[j] - timestamp - 1.0)) <= 0.03:
            relative.append(np.linalg.norm((aligned[j] - aligned[i]) - (truth[j] - truth[i])))
    relative_array = np.asarray(relative, dtype=np.float64)
    finite_gt = np.isfinite(features[:, 7]) if len(features) else np.zeros(0, dtype=bool)
    return {
        "frames": len(timestamps),
        "duration_s": float(timestamps[-1] - timestamps[0]),
        "ate_rmse_m": float(np.sqrt(np.mean(ate * ate))),
        "ate_median_m": float(np.median(ate)),
        "ate_p95_m": float(np.quantile(ate, 0.95)),
        "rpe_global_1s_rmse_m": float(np.sqrt(np.mean(relative_array * relative_array))),
        "feature_candidates": len(features),
        "feature_gate_pass_rate": float(np.mean(features[:, 11] > 0.5)) if len(features) else float("nan"),
        "finite_gt_feature_targets": int(finite_gt.sum()),
    }


def _lookup_rate(log_path: Path) -> float | None:
    if not log_path.exists():
        return None
    match = re.search(r"sigma lookup hits=(\d+) queries=(\d+) hit_rate=([0-9.eE+-]+)",
                      log_path.read_text(errors="replace"))
    return None if match is None else float(match.group(3))


def _live_neta_predictions(log_path: Path) -> int | None:
    if not log_path.exists():
        return None
    match = re.search(
        r"live Net-A batches=(\d+) predictions=(\d+)",
        log_path.read_text(errors="replace"),
    )
    return None if match is None else int(match.group(2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sequences", nargs="+", default=list(TEST_SEQUENCES))
    parser.add_argument("--arms", nargs="+", choices=ARMS, default=list(ARMS))
    parser.add_argument(
        "--min-hit-rate", type=float, default=0.99,
        help="minimum exact visual sidecar lookup rate for every non-stock run",
    )
    parser.add_argument(
        "--require-live-neta", action="store_true",
        help="validate causal live Net-A inference instead of offline sidecar lookup",
    )
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    lookup_failures = []
    for sequence in args.sequences:
        for arm in args.arms:
            path = args.run_dir / f"{sequence}_{arm}.h5"
            if not path.exists():
                raise FileNotFoundError(path)
            row = {
                "sequence": sequence,
                "split": SPLIT_BY_SEQUENCE[sequence],
                "arm": arm,
                **_metrics(path),
            }
            row["sigma_lookup_hit_rate"] = (
                1.0 if arm == "stock"
                else _lookup_rate(args.log_dir / f"{sequence}_{arm}.log")
            )
            row["live_neta_predictions"] = (
                0 if arm == "stock"
                else _live_neta_predictions(args.log_dir / f"{sequence}_{arm}.log")
            )
            if arm != "stock":
                if args.require_live_neta:
                    if row["live_neta_predictions"] is None or row["live_neta_predictions"] <= 0:
                        lookup_failures.append({
                            "sequence": sequence,
                            "arm": arm,
                            "live_neta_predictions": row["live_neta_predictions"],
                            "failure": "missing live Net-A inference evidence",
                        })
                elif (
                    row["sigma_lookup_hit_rate"] is None
                    or row["sigma_lookup_hit_rate"] < args.min_hit_rate
                ):
                    lookup_failures.append({
                        "sequence": sequence,
                        "arm": arm,
                        "hit_rate": row["sigma_lookup_hit_rate"],
                        "minimum": args.min_hit_rate,
                    })
            rows.append(row)
            print(json.dumps(row, sort_keys=True))
    csv_path = args.out_dir / "stage3_benchmark_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    primary_rows = [row for row in rows if row["split"] == "test"]
    primary_path = args.out_dir / "stage3_primary_test_summary.csv"
    with primary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(primary_rows)
    supplementary_path = args.out_dir / "stage3_all11_supplementary_summary.csv"
    with supplementary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    stock = {row["sequence"]: row for row in rows if row["arm"] == "stock"}
    comparisons = []
    for row in rows:
        baseline = stock[row["sequence"]]
        comparisons.append({
            "sequence": row["sequence"],
            "split": row["split"],
            "arm": row["arm"],
            "ate_ratio_to_stock": row["ate_rmse_m"] / baseline["ate_rmse_m"],
            "rpe_ratio_to_stock": row["rpe_global_1s_rmse_m"] / baseline["rpe_global_1s_rmse_m"],
        })
    (args.out_dir / "stage3_relative_to_stock.json").write_text(
        json.dumps(comparisons, indent=2) + "\n", encoding="utf-8"
    )
    lookup_report = {
        "inference_mode": (
            "causal live batch Net-A"
            if args.require_live_neta
            else "offline exact timestamp_us and feature_id"
        ),
        "minimum_hit_rate": None if args.require_live_neta else args.min_hit_rate,
        "passed": not lookup_failures,
        "failures": lookup_failures,
    }
    (args.out_dir / "stage3_sigma_lookup_validation.json").write_text(
        json.dumps(lookup_report, indent=2) + "\n", encoding="utf-8"
    )
    if lookup_failures:
        raise RuntimeError(
            f"{len(lookup_failures)} runs failed visual-inference validation; "
            "trajectory comparisons are invalid"
        )


if __name__ == "__main__":
    main()
