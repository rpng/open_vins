#!/usr/bin/env python3
"""Export the accepted PyTorch Net-A checkpoint to portable inference arrays."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stage2_train"))
from net_a_visual_deepsets import NetA


def load_checkpoint(path: Path) -> dict:
    try:
        return torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        return torch.load(path, map_location="cpu")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    checkpoint = load_checkpoint(args.checkpoint)
    if checkpoint.get("head") != "A":
        raise ValueError("checkpoint is not a Net-A checkpoint")
    net = NetA()
    net.load_state_dict(checkpoint["model_state_dict"])
    net.eval()

    verification_features = torch.tensor(
        [
            [1.10, 0.0, 0.20, 0.30, 0.45, 0.80, 1.20, 1.0],
            [2.00, 1.0, 0.75, 0.55, 1.10, 1.40, 1.80, 1.0],
            [2.70, 0.0, 0.50, 0.85, 0.20, 0.30, 0.90, 1.0],
        ],
        dtype=torch.float32,
    )
    verification_context = torch.tensor(
        [5.1, 1.4, 0.48, 0.23, 1.39, 0.67], dtype=torch.float32
    )
    with torch.no_grad():
        verification_log_sigma = net(
            verification_features.unsqueeze(0),
            torch.ones((1, len(verification_features)), dtype=torch.float32),
            verification_context.unsqueeze(0),
        ).squeeze(0)

    state = net.state_dict()
    arrays = {
        "encoder0_weight": state["encoder.net.0.weight"].numpy(),
        "encoder0_bias": state["encoder.net.0.bias"].numpy(),
        "encoder2_weight": state["encoder.net.2.weight"].numpy(),
        "encoder2_bias": state["encoder.net.2.bias"].numpy(),
        "pool_weight": state["pool_proj.weight"].numpy(),
        "pool_bias": state["pool_proj.bias"].numpy(),
        "head0_weight": state["head.0.weight"].numpy(),
        "head0_bias": state["head.0.bias"].numpy(),
        "head2_weight": state["head.2.weight"].numpy(),
        "head2_bias": state["head.2.bias"].numpy(),
        "verification_features": verification_features.numpy(),
        "verification_frame_context": verification_context.numpy(),
        "verification_log_sigma": verification_log_sigma.numpy(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    print(
        f"[neta-export] PASS output={args.output} "
        f"parameters={sum(value.numel() for value in state.values())}"
    )


if __name__ == "__main__":
    main()
