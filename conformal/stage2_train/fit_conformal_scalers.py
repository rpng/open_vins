#!/usr/bin/env python3
"""Fit sequence-disjoint visual and inertial conformal multipliers.

No test sequence is opened. Net A supplies per-feature visual sigma. The
accepted Net-B checkpoint selected epoch zero, so inertial sigma is the
fit-only constant four-channel correction recorded in that checkpoint.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from hdf5_dump_dataset import EUROC_SEQUENCE_DISJOINT_SPLIT, NetADump, collate_variable_k
from net_a_visual_deepsets import NetA
from split_conformal_per_modality import split_conformal_quantile


BASE_SIGMAS = np.asarray((1.6968e-4, 2.0e-3, 1.9393e-5, 3.0e-3), dtype=np.float64)
CHANNELS = ("sigma_w", "sigma_a", "sigma_wb", "sigma_ab")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_checkpoint(path: Path) -> dict:
    try:
        return torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        return torch.load(path, map_location="cpu")


@torch.no_grad()
def _visual_scores(array_dir: Path, checkpoint: dict, sequences: list[str]) -> dict[str, np.ndarray]:
    net = NetA()
    net.load_state_dict(checkpoint["model_state_dict"])
    net.eval()
    result: dict[str, np.ndarray] = {}
    for sequence in sequences:
        dataset = NetADump(array_dir, [sequence])
        loader = DataLoader(
            dataset, batch_size=32, shuffle=False, collate_fn=collate_variable_k
        )
        blocks = []
        for features, mask, context, errors in loader:
            log_sigma = net(features, mask, context).clamp(-7.0, 7.0)
            valid = mask.bool()
            blocks.append(
                (errors[valid].numpy() / torch.exp(log_sigma[valid]).numpy()).astype(np.float64)
            )
        result[sequence] = np.concatenate(blocks)
    return result


def _inertial_scores(
    array_dir: Path, checkpoint: dict, sequences: list[str]
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    if int(checkpoint.get("selected_epoch", -1)) != 0:
        raise ValueError("guarded Net-B checkpoint did not select epoch zero")
    log_scales = np.asarray(checkpoint["constant_log_scales"], dtype=np.float64)
    correction = np.exp(log_scales)
    result = {}
    for sequence in sequences:
        with np.load(array_dir / f"{sequence}_stage2_netb.npz") as archive:
            target = archive["target_normalized"].astype(np.float64)
        result[sequence] = (target / correction[None, :]).reshape(-1)
    return result, correction


def _grouped_block_bootstrap_ci(
    score_groups: dict[str, np.ndarray], alpha: float, block_len: int,
    n_boot: int, seed: int
) -> tuple[float, float]:
    rng = np.random.default_rng(seed)
    quantiles = np.empty(n_boot, dtype=np.float64)
    for sample in range(n_boot):
        boot_groups = []
        for scores in score_groups.values():
            n = len(scores)
            length = min(block_len, n)
            count = int(np.ceil(n / length))
            starts = rng.integers(0, n, size=count)
            offsets = np.arange(length)
            indices = ((starts[:, None] + offsets[None, :]) % n).reshape(-1)[:n]
            boot_groups.append(scores[indices])
        boot = np.concatenate(boot_groups)
        quantiles[sample] = split_conformal_quantile(
            boot, np.ones_like(boot), alpha
        )
    return tuple(float(x) for x in np.quantile(quantiles, (0.025, 0.975)))


def _modality_report(
    score_groups: dict[str, np.ndarray], alpha: float, block_len: int,
    n_boot: int, seed: int
) -> dict[str, object]:
    scores = np.concatenate(list(score_groups.values()))
    q_alpha = split_conformal_quantile(scores, np.ones_like(scores), alpha)
    pre = {sequence: float(np.mean(values <= 1.0))
           for sequence, values in score_groups.items()}
    post = {sequence: float(np.mean(values <= q_alpha))
            for sequence, values in score_groups.items()}
    ci = _grouped_block_bootstrap_ci(score_groups, alpha, block_len, n_boot, seed)
    return {
        "n_scores": int(len(scores)),
        "q_alpha": q_alpha,
        "q_alpha_block_bootstrap_95_ci": list(ci),
        "bootstrap_block_length_scores": block_len,
        "bootstrap_replicates": n_boot,
        "coverage_before": float(np.mean(scores <= 1.0)),
        "coverage_after": float(np.mean(scores <= q_alpha)),
        "per_sequence_coverage_before": pre,
        "per_sequence_coverage_after": post,
        "score_median": float(np.median(scores)),
        "score_p95": float(np.quantile(scores, 0.95)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--array-dir", type=Path, required=True)
    parser.add_argument("--net-a", type=Path, required=True)
    parser.add_argument("--net-b", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--alpha", type=float, default=0.1)
    parser.add_argument("--bootstrap-replicates", type=int, default=1000)
    args = parser.parse_args()
    if not (0.0 < args.alpha < 1.0):
        raise ValueError("alpha must lie strictly between zero and one")

    split = EUROC_SEQUENCE_DISJOINT_SPLIT
    net_a_checkpoint = _load_checkpoint(args.net_a)
    net_b_checkpoint = _load_checkpoint(args.net_b)
    visual = _visual_scores(args.array_dir, net_a_checkpoint, split.calibration)
    inertial, inertial_correction = _inertial_scores(
        args.array_dir, net_b_checkpoint, split.calibration
    )
    visual_report = _modality_report(
        visual, args.alpha, block_len=50,
        n_boot=args.bootstrap_replicates, seed=17
    )
    inertial_report = _modality_report(
        inertial, args.alpha, block_len=200,  # 50 intervals x four channels
        n_boot=args.bootstrap_replicates, seed=29
    )
    inertial_report["constant_log_scales"] = np.log(inertial_correction).tolist()
    inertial_report["constant_multipliers"] = inertial_correction.tolist()
    inertial_report["base_sigmas"] = dict(zip(CHANNELS, BASE_SIGMAS.tolist()))
    inertial_report["corrected_sigmas_before_conformal"] = dict(
        zip(CHANNELS, (BASE_SIGMAS * inertial_correction).tolist())
    )
    inertial_report["corrected_sigmas_after_conformal"] = dict(
        zip(
            CHANNELS,
            (BASE_SIGMAS * inertial_correction * float(inertial_report["q_alpha"])).tolist(),
        )
    )
    report = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "alpha": args.alpha,
        "target_coverage": 1.0 - args.alpha,
        "calibration_sequences": split.calibration,
        "test_sequences_not_opened": split.test,
        "net_a_checkpoint": str(args.net_a),
        "net_a_sha256": _sha256(args.net_a),
        "net_b_checkpoint": str(args.net_b),
        "net_b_sha256": _sha256(args.net_b),
        "net_b_interpretation": "epoch-zero fit-only constant correction; conditional TCN rejected",
        "modalities": {
            "visual": visual_report,
            "inertial": inertial_report,
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
