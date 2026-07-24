#!/usr/bin/env python3
"""Run accepted Stage-2 models over fixed test inputs and emit NPZ sidecars."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stage2_train"))
from net_a_visual_deepsets import NetA


ARMS = ("stock", "learned", "conformalised", "oracle")
BASE_SIGMAS = np.asarray((1.6968e-4, 2.0e-3, 1.9393e-5, 3.0e-3), dtype=np.float64)


def _load_checkpoint(path: Path) -> dict:
    try:
        return torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        return torch.load(path, map_location="cpu")


@torch.no_grad()
def _predict_visual(inputs: dict[str, np.ndarray], checkpoint: dict) -> np.ndarray:
    net = NetA()
    net.load_state_dict(checkpoint["model_state_dict"])
    net.eval()
    features = torch.from_numpy(inputs["visual_features"])
    contexts = torch.from_numpy(inputs["visual_frame_context"])
    offsets = inputs["visual_frame_offsets"]
    output = np.empty(len(features), dtype=np.float64)
    for frame in range(len(contexts)):
        start, stop = int(offsets[frame]), int(offsets[frame + 1])
        frame_features = features[start:stop].unsqueeze(0)
        mask = torch.ones((1, stop - start), dtype=torch.float32)
        log_sigma = net(frame_features, mask, contexts[frame].unsqueeze(0)).clamp(-7.0, 7.0)
        output[start:stop] = torch.exp(log_sigma.squeeze(0)).numpy()
    return output


def build_sequence(
    input_path: Path, net_a_path: Path, net_b_path: Path,
    conformal_path: Path, output_dir: Path, sequence: str
) -> None:
    with np.load(input_path) as archive:
        inputs = {name: archive[name] for name in archive.files}
    net_a = _load_checkpoint(net_a_path)
    net_b = _load_checkpoint(net_b_path)
    if int(net_b.get("selected_epoch", -1)) != 0:
        raise ValueError("Net-B checkpoint is not the accepted epoch-zero constant model")
    conformal = json.loads(conformal_path.read_text(encoding="utf-8"))
    q_visual = float(conformal["modalities"]["visual"]["q_alpha"])
    q_inertial = float(conformal["modalities"]["inertial"]["q_alpha"])
    inertial_correction = np.exp(np.asarray(net_b["constant_log_scales"], dtype=np.float64))
    visual_learned = _predict_visual(inputs, net_a)

    for arm in ARMS:
        if arm == "stock":
            visual_sigma = np.ones(len(visual_learned), dtype=np.float64)
            imu_sigma = np.broadcast_to(BASE_SIGMAS, (len(inputs["frame_timestamp"]), 4)).copy()
        elif arm == "learned":
            visual_sigma = visual_learned
            imu_sigma = np.broadcast_to(
                BASE_SIGMAS * inertial_correction,
                (len(inputs["frame_timestamp"]), 4),
            ).copy()
        elif arm == "conformalised":
            visual_sigma = visual_learned * q_visual
            imu_sigma = np.broadcast_to(
                BASE_SIGMAS * inertial_correction * q_inertial,
                (len(inputs["frame_timestamp"]), 4),
            ).copy()
        else:
            gt_visual = inputs["gt_visual_error"].astype(np.float64)
            valid_visual = np.isfinite(gt_visual) & (gt_visual > 1e-6) & (gt_visual <= np.hypot(752.0, 480.0))
            visual_sigma = np.where(valid_visual, gt_visual, 1.0)
            imu_sigma = np.broadcast_to(BASE_SIGMAS, (len(inputs["frame_timestamp"]), 4)).copy()
            valid_imu = inputs["inertial_oracle_valid"].astype(bool)
            oracle_imu = inputs["inertial_oracle_density"].astype(np.float64)
            imu_sigma[valid_imu] = np.maximum(oracle_imu[valid_imu], 1e-12)

        np.savez_compressed(
            output_dir / f"{sequence}_{arm}_sigma.npz",
            arm=np.asarray(arm),
            sequence=np.asarray(sequence),
            frame_timestamp=inputs["frame_timestamp"],
            imu_sigmas=imu_sigma,
            feature_timestamp=inputs["feature_timestamp"],
            feature_id=inputs["feature_id"],
            visual_sigmas=visual_sigma,
        )
        print(sequence, arm, "frames", len(imu_sigma), "features", len(visual_sigma))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--net-a", type=Path, required=True)
    parser.add_argument("--net-b", type=Path, required=True)
    parser.add_argument("--conformal", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--sequences", nargs="+", default=["MH_05_difficult", "V2_02_medium", "V2_03_difficult"])
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for sequence in args.sequences:
        build_sequence(
            args.input_dir / f"{sequence}_stage3_inputs.npz",
            args.net_a, args.net_b, args.conformal, args.out_dir, sequence,
        )


if __name__ == "__main__":
    main()
