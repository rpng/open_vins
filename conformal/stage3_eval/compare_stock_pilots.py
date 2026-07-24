#!/usr/bin/env python3
"""Check that the refactored live-inference runner preserves stock behavior."""

from __future__ import annotations

import argparse
from pathlib import Path

from evaluate_stage3_runs import _metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--max-ate-ratio", type=float, default=1.20)
    parser.add_argument("--max-frame-difference", type=int, default=5)
    args = parser.parse_args()

    reference = _metrics(args.reference)
    candidate = _metrics(args.candidate)
    ratio = candidate["ate_rmse_m"] / reference["ate_rmse_m"]
    symmetric_ratio = max(ratio, 1.0 / ratio)
    frame_difference = abs(candidate["frames"] - reference["frames"])
    print(
        "[stage3-stock-parity]"
        f" reference_ate_rmse_m={reference['ate_rmse_m']:.9f}"
        f" candidate_ate_rmse_m={candidate['ate_rmse_m']:.9f}"
        f" ate_ratio={ratio:.9f}"
        f" symmetric_ate_ratio={symmetric_ratio:.9f}"
        f" frame_difference={frame_difference}"
    )
    if symmetric_ratio > args.max_ate_ratio:
        raise RuntimeError(
            f"symmetric stock ATE ratio {symmetric_ratio:.6f} exceeds "
            f"{args.max_ate_ratio:.6f}"
        )
    if frame_difference > args.max_frame_difference:
        raise RuntimeError(
            f"stock frame difference {frame_difference} exceeds "
            f"{args.max_frame_difference}"
        )
    print("[stage3-stock-parity] PASS")


if __name__ == "__main__":
    main()
