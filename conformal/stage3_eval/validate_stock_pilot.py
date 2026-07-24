#!/usr/bin/env python3
"""Reject a structurally valid but scientifically divergent stock pilot."""

from __future__ import annotations

import argparse
from pathlib import Path

from evaluate_stage3_runs import _metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dump", type=Path, required=True)
    parser.add_argument("--max-ate-rmse-m", type=float, default=1.0)
    parser.add_argument("--min-feature-candidates", type=int, default=1000)
    args = parser.parse_args()

    metrics = _metrics(args.dump)
    print(
        "[stage3-stock-sanity]"
        f" frames={metrics['frames']}"
        f" features={metrics['feature_candidates']}"
        f" ate_rmse_m={metrics['ate_rmse_m']:.9f}"
        f" rpe_global_1s_rmse_m={metrics['rpe_global_1s_rmse_m']:.9f}"
    )
    failures = []
    if metrics["ate_rmse_m"] > args.max_ate_rmse_m:
        failures.append(
            f"ATE RMSE {metrics['ate_rmse_m']:.6f} m exceeds "
            f"{args.max_ate_rmse_m:.6f} m"
        )
    if metrics["feature_candidates"] < args.min_feature_candidates:
        failures.append(
            f"only {metrics['feature_candidates']} feature candidates; expected at least "
            f"{args.min_feature_candidates}"
        )
    if failures:
        raise RuntimeError("; ".join(failures))
    print("[stage3-stock-sanity] PASS")


if __name__ == "__main__":
    main()
