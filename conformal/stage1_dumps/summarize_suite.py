#!/usr/bin/env python3
"""Validate and summarize a directory of EuRoC Stage-1 HDF5 dumps."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import h5py
import numpy as np

from validate_dump import validate


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values: np.ndarray, quantile: float) -> float:
    finite = values[np.isfinite(values)]
    return float(np.percentile(finite, quantile)) if finite.size else float("nan")


def summarize_dump(path: Path) -> dict[str, object]:
    validated = validate(path)
    with h5py.File(path, "r") as dump:
        features = dump["/features/diagnostics"][:]
        frames = dump["/frames/diagnostics"][:]
        covariance = dump["/frames/covariance_imu15"][:]

    runtime_path = path.with_name(path.stem + ".runtime_seconds.txt")
    runtime = runtime_path.read_text().strip() if runtime_path.exists() else ""
    finite_gt = np.isfinite(features[:, 7])
    covariance_diagonal = np.diagonal(covariance, axis1=1, axis2=2)
    return {
        "sequence": validated["sequence"],
        "frames": validated["frames"],
        "feature_candidates": validated["features"],
        "feature_accepted": int(np.count_nonzero(features[:, 11] == 1.0)),
        "feature_rejected": int(np.count_nonzero(features[:, 11] == 0.0)),
        "gate_pass_rate": validated["gate_pass_rate"],
        "duration_s": validated["duration_s"],
        "wall_runtime_s": runtime,
        "mean_tracked_features": float(np.mean(frames[:, 1])),
        "mean_lost_features": float(np.mean(frames[:, 2])),
        "filter_residual_median_px": percentile(features[:, 6], 50),
        "filter_residual_p95_px": percentile(features[:, 6], 95),
        "gt_reprojection_finite_fraction": validated["gt_reprojection_finite_fraction"],
        "gt_reprojection_median_px": percentile(features[finite_gt, 7], 50),
        "gt_reprojection_p95_px": percentile(features[finite_gt, 7], 95),
        "chi2_median": percentile(features[:, 8], 50),
        "chi2_p95": percentile(features[:, 8], 95),
        "covariance_symmetry_max": validated["covariance_symmetry_max"],
        "covariance_min_diagonal": float(np.min(covariance_diagonal)),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", type=Path)
    parser.add_argument("--expected-count", type=int, default=11)
    parser.add_argument("--image-id", default="")
    args = parser.parse_args()

    dumps = sorted(args.results_dir.glob("*_stage1.h5"))
    if len(dumps) != args.expected_count:
        raise SystemExit(
            f"expected {args.expected_count} canonical dumps, found {len(dumps)} in {args.results_dir}"
        )

    rows = [summarize_dump(path) for path in dumps]
    summary_path = args.results_dir / "stage1_suite_summary.csv"
    with summary_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    metadata = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "image_id": args.image_id,
        "sequence_count": len(rows),
        "total_frames": sum(int(row["frames"]) for row in rows),
        "total_feature_candidates": sum(int(row["feature_candidates"]) for row in rows),
        "all_validated": True,
    }
    metadata_path = args.results_dir / "stage1_suite_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")

    checksum_path = args.results_dir / "stage1_suite_checksums.sha256"
    artifacts = sorted(
        path
        for path in args.results_dir.iterdir()
        if path.is_file()
        and path.name != checksum_path.name
        and (
            path.name.startswith(tuple(str(row["sequence"]) for row in rows))
            or path in (summary_path, metadata_path)
        )
    )
    checksum_path.write_text(
        "".join(f"{sha256(path)}  {path.name}\n" for path in artifacts)
    )

    print(f"[stage1-summary] PASS sequences={len(rows)}")
    print(f"[stage1-summary] total_frames={metadata['total_frames']}")
    print(f"[stage1-summary] total_feature_candidates={metadata['total_feature_candidates']}")
    print(f"[stage1-summary] summary={summary_path}")
    print(f"[stage1-summary] checksums={checksum_path}")


if __name__ == "__main__":
    main()
