#!/usr/bin/env python3
"""
train_heads.py  --  offline trainer for Net A and Net B.

Companion to: Section 8.3 ("Training: fully offline, no gradients through the filter") and
              Section 17 (the decoupled pipeline: Stage 2 iterates on a laptop in minutes).

This is Stage 2. It reads the Stage-1 HDF5 dumps, trains each head with the heteroscedastic
NLL, and freezes the weights. NOTHING here touches C++ or OpenVINS -- the whole point of the
decoupled design (Section 17) is that Stage 1 runs once and Stage 2 can be re-run hundreds of
times without recompiling anything. "Minutes on a laptop" (Section 8.3).

Pipeline (Section 8.3):
    1. (Stage 1, already done) run unmodified OpenVINS, dump diagnostics+residuals+GT to HDF5.
    2. Compute the error each measurement actually made from motion-capture GT.
    3. Train each head to predict that error via heteroscedastic NLL.        <- THIS FILE
    4. Freeze; plug predicted sigma back in as Q and R.                       (Stage 3)

Ground truth is used ONLY here in training; it is never needed at inference (Section 8.3).

Usage:
    python train_heads.py --head A --h5-dir /data/conformal_dumps --out netA.pt
    python train_heads.py --head B --h5-dir /data/conformal_dumps --out netB.pt

Checkpoints include preprocessing statistics and the exact sequence split, not just weights,
so Stage 3 cannot silently apply a model with different input scaling.
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from heteroscedastic_nll import heteroscedastic_nll
from hdf5_dump_dataset import (EUROC_SEQUENCE_DISJOINT_SPLIT, NetADump, NetBDump, collate_variable_k)
from net_a_visual_deepsets import NetA
from net_b_imu_dilated_tcn import NetB

NETB_SELECTION_FIT = ["MH_01_easy", "MH_02_easy", "V1_01_easy", "V1_02_medium"]
NETB_SELECTION_VALIDATION = ["MH_03_medium", "V2_01_easy"]
NETB_EARLY_STOPPING_PATIENCE = 10
NETB_WEIGHT_DECAY = 1e-4


def _seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _split_dict() -> dict[str, list[str]]:
    split = EUROC_SEQUENCE_DISJOINT_SPLIT
    return {"train": split.train, "calibration": split.calibration, "test": split.test}


def _save_checkpoint(path: str, payload: dict) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    torch.save(payload, destination)


@torch.no_grad()
def _eval_net_a(net: NetA, loader: DataLoader, device: str) -> float:
    net.eval()
    weighted_loss = 0.0
    weight = 0.0
    for feats, mask, ctx, err in loader:
        mask = mask.to(device)
        loss = heteroscedastic_nll(
            err.to(device), net(feats.to(device), mask, ctx.to(device)), mask=mask
        )
        count = float(mask.sum())
        weighted_loss += float(loss) * count
        weight += count
    return weighted_loss / max(weight, 1.0)


def train_net_a(h5_dir: str, out_path: str, epochs: int, lr: float,
                device: str, seed: int) -> None:
    split = EUROC_SEQUENCE_DISJOINT_SPLIT
    train_ds = NetADump(h5_dir, split.train)
    calibration_ds = NetADump(h5_dir, split.calibration)
    loader = DataLoader(train_ds, batch_size=16, shuffle=True, collate_fn=collate_variable_k)
    calibration_loader = DataLoader(
        calibration_ds, batch_size=32, shuffle=False, collate_fn=collate_variable_k
    )
    net = NetA().to(device)
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    history = []
    for epoch in range(epochs):
        net.train()
        train_total = 0.0
        train_weight = 0.0
        for feats, mask, ctx, err in loader:  # shapes: [B,K,D],[B,K],[B,C],[B,K]
            opt.zero_grad()
            log_sigma = net(feats.to(device), mask.to(device), ctx.to(device))
            loss = heteroscedastic_nll(err.to(device), log_sigma, mask=mask.to(device))
            loss.backward()
            torch.nn.utils.clip_grad_norm_(net.parameters(), max_norm=5.0)
            opt.step()
            count = float(mask.sum())
            train_total += float(loss.detach()) * count
            train_weight += count
        train_nll = train_total / max(train_weight, 1.0)
        history.append({"epoch": epoch + 1, "train_nll": train_nll})
        print(json.dumps(history[-1]))
    # Evaluate once after freezing. Never select an epoch/checkpoint with the
    # conformal-calibration pool: doing so would leak calibration information.
    calibration_nll = _eval_net_a(net, calibration_loader, device)
    print(json.dumps({"frozen_calibration_nll": calibration_nll}))
    _save_checkpoint(out_path, {
        "head": "A",
        "model_state_dict": {key: value.detach().cpu() for key, value in net.state_dict().items()},
        "frozen_calibration_nll": calibration_nll,
        "history": history,
        "split": _split_dict(),
        "seed": seed,
        "target": "gt_reprojection_residual_norm_px",
        "feature_schema": [
            "log1p(track_measurements)", "last_camera", "u/752", "v/480",
            "log1p(filter_residual_norm)", "log1p(chi2)",
            "log1p(chi2_threshold)", "sigma_pix",
        ],
        "frame_context_schema": [
            "log1p(num_tracked)", "log1p(num_lost)", "mean_brightness/255",
            "state_dim/1000", "log1p(candidate_count)", "gate_pass_ratio",
        ],
    })


@torch.no_grad()
def _eval_net_b(net: NetB, loader: DataLoader, device: str,
                input_mean: torch.Tensor, input_std: torch.Tensor) -> float:
    net.eval()
    total = 0.0
    count = 0
    for imu_window, err in loader:
        imu_window = (imu_window.to(device) - input_mean) / input_std
        loss = heteroscedastic_nll(err.to(device), net(imu_window))
        total += float(loss) * len(imu_window)
        count += len(imu_window)
    return total / max(count, 1)


def _netb_input_stats(dataset: NetBDump, device: str) -> tuple[np.ndarray, np.ndarray, torch.Tensor, torch.Tensor]:
    windows = np.stack(dataset.windows)
    mean_np = windows.mean(axis=(0, 1), dtype=np.float64).astype(np.float32)
    std_np = windows.std(axis=(0, 1), dtype=np.float64).astype(np.float32)
    std_np = np.maximum(std_np, 1e-6)
    mean = torch.from_numpy(mean_np).view(1, 1, 6).to(device)
    std = torch.from_numpy(std_np).view(1, 1, 6).to(device)
    return mean_np, std_np, mean, std


def _train_netb_epoch(net: NetB, loader: DataLoader, opt: torch.optim.Optimizer,
                      device: str, input_mean: torch.Tensor,
                      input_std: torch.Tensor) -> float:
    net.train()
    total = 0.0
    count = 0
    for imu_window, err in loader:
        opt.zero_grad()
        normalized_window = (imu_window.to(device) - input_mean) / input_std
        log_scales = net(normalized_window)
        loss = heteroscedastic_nll(err.to(device), log_scales)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(net.parameters(), max_norm=5.0)
        opt.step()
        total += float(loss.detach()) * len(imu_window)
        count += len(imu_window)
    return total / max(count, 1)


def _initialize_netb_constant(net: NetB, dataset: NetBDump, device: str) -> np.ndarray:
    targets = np.stack(dataset.targets).astype(np.float64)
    log_rms = (0.5 * np.log(np.maximum(np.mean(targets * targets, axis=0), 1e-12))).astype(np.float32)
    final_linear = net.head[-1]
    with torch.no_grad():
        final_linear.weight.zero_()
        final_linear.bias.copy_(torch.from_numpy(log_rms).to(device))
    return log_rms


def _constant_netb_nll(dataset: NetBDump, log_scales: np.ndarray) -> float:
    targets = np.stack(dataset.targets).astype(np.float64)
    scales = np.asarray(log_scales, dtype=np.float64)
    return float(np.mean(0.5 * targets * targets * np.exp(-2.0 * scales) + scales))


def train_net_b(h5_dir: str, sidecar_dir: str, out_path: str, epochs: int,
                lr: float, device: str, seed: int) -> None:
    split = EUROC_SEQUENCE_DISJOINT_SPLIT
    # Select training duration without touching the conformal-calibration pool.
    selection_train_ds = NetBDump(h5_dir, NETB_SELECTION_FIT, sidecar_dir)
    selection_validation_ds = NetBDump(
        h5_dir, NETB_SELECTION_VALIDATION, sidecar_dir
    )
    selection_loader = DataLoader(selection_train_ds, batch_size=64, shuffle=True)
    selection_validation_loader = DataLoader(
        selection_validation_ds, batch_size=128, shuffle=False
    )
    _, _, selection_mean, selection_std = _netb_input_stats(selection_train_ds, device)
    selection_net = NetB(use_std_pool=True).to(device)
    constant_log_scales = _initialize_netb_constant(selection_net, selection_train_ds, device)
    selection_opt = torch.optim.Adam(
        selection_net.parameters(), lr=lr, weight_decay=NETB_WEIGHT_DECAY
    )
    epoch_zero_validation_nll = _eval_net_b(
        selection_net, selection_validation_loader, device,
        selection_mean, selection_std
    )
    selection_history = [{
        "stage": "epoch_selection",
        "epoch": 0,
        "train_nll": _constant_netb_nll(selection_train_ds, constant_log_scales),
        "training_pool_validation_nll": epoch_zero_validation_nll,
    }]
    print(json.dumps(selection_history[0]))
    best_epoch = 0
    best_validation_nll = epoch_zero_validation_nll
    best_state = {
        key: value.detach().cpu().clone()
        for key, value in selection_net.state_dict().items()
    }
    stale_epochs = 0
    for epoch in range(epochs):
        train_nll = _train_netb_epoch(
            selection_net, selection_loader, selection_opt, device,
            selection_mean, selection_std
        )
        validation_nll = _eval_net_b(
            selection_net, selection_validation_loader, device,
            selection_mean, selection_std
        )
        record = {
            "stage": "epoch_selection",
            "epoch": epoch + 1,
            "train_nll": train_nll,
            "training_pool_validation_nll": validation_nll,
        }
        selection_history.append(record)
        print(json.dumps(record))
        if validation_nll < best_validation_nll:
            best_validation_nll = validation_nll
            best_epoch = epoch + 1
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in selection_net.state_dict().items()
            }
            stale_epochs = 0
        else:
            stale_epochs += 1
        if stale_epochs >= NETB_EARLY_STOPPING_PATIENCE:
            print(json.dumps({
                "stage": "early_stopping",
                "epoch": epoch + 1,
                "patience": NETB_EARLY_STOPPING_PATIENCE,
            }))
            break

    # Retain the actual validation-selected weights. A prior attempt to transfer
    # only the epoch count into an all-sequence refit failed under sequence shift.
    selection_net.load_state_dict(best_state)
    calibration_ds = NetBDump(h5_dir, split.calibration, sidecar_dir)
    calibration_loader = DataLoader(calibration_ds, batch_size=128, shuffle=False)
    calibration_nll = _eval_net_b(
        selection_net, calibration_loader, device, selection_mean, selection_std
    )
    calibration_constant_nll = _constant_netb_nll(
        calibration_ds, constant_log_scales
    )
    print(json.dumps({
        "selected_epoch": best_epoch,
        "training_pool_best_validation_nll": best_validation_nll,
        "frozen_calibration_nll": calibration_nll,
        "training_fit_constant_calibration_nll": calibration_constant_nll,
    }))
    _save_checkpoint(out_path, {
        "head": "B",
        "model_state_dict": best_state,
        "frozen_calibration_nll": calibration_nll,
        "training_fit_constant_calibration_nll": calibration_constant_nll,
        "selected_epoch": best_epoch,
        "training_pool_best_validation_nll": best_validation_nll,
        "constant_log_scales": constant_log_scales,
        "selection_fit_sequences": NETB_SELECTION_FIT,
        "selection_validation_sequences": NETB_SELECTION_VALIDATION,
        "selection_history": selection_history,
        "early_stopping_patience": NETB_EARLY_STOPPING_PATIENCE,
        "weight_decay": NETB_WEIGHT_DECAY,
        "split": _split_dict(),
        "seed": seed,
        "input_mean": selection_mean.detach().cpu().numpy().reshape(6),
        "input_std": selection_std.detach().cpu().numpy().reshape(6),
        "target": "preintegration_density_divided_by_stock_sigma",
        "target_order": ["sigma_w", "sigma_a", "sigma_wb", "sigma_ab"],
        "base_sigmas": np.asarray((1.6968e-4, 2.0e-3, 1.9393e-5, 3.0e-3), dtype=np.float32),
    })


def main() -> None:
    ap = argparse.ArgumentParser(description="conformal Stage-2 offline head trainer")
    ap.add_argument("--head", choices=["A", "B"], required=True)
    ap.add_argument("--h5-dir", required=True, help="dir of Stage-1 <seq>.h5 dumps")
    ap.add_argument("--sidecar-dir", help="dir of Stage-2 Net-B sidecars (defaults to --h5-dir)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()
    _seed_everything(args.seed)
    if args.head == "A":
        train_net_a(args.h5_dir, args.out, args.epochs, args.lr, args.device, args.seed)
    else:
        train_net_b(
            args.h5_dir, args.sidecar_dir or args.h5_dir, args.out,
            args.epochs, args.lr, args.device, args.seed
        )


if __name__ == "__main__":
    main()
