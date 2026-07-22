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

TODO(intern): wire the DataLoaders (hdf5_dump_dataset.py), the optimizer loop, checkpointing,
and a validation pass on the sequence-disjoint calibration split.
"""

from __future__ import annotations

import argparse

import torch
from torch.utils.data import DataLoader

from heteroscedastic_nll import heteroscedastic_nll
from hdf5_dump_dataset import (EUROC_SEQUENCE_DISJOINT_SPLIT, NetADump, NetBDump, collate_variable_k)
from net_a_visual_deepsets import NetA
from net_b_imu_dilated_tcn import NetB


def train_net_a(h5_dir: str, out_path: str, epochs: int, lr: float, device: str) -> None:
    split = EUROC_SEQUENCE_DISJOINT_SPLIT
    ds = NetADump(h5_dir, split.train)
    loader = DataLoader(ds, batch_size=16, shuffle=True, collate_fn=collate_variable_k)
    net = NetA().to(device)
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    for epoch in range(epochs):
        for feats, mask, ctx, err in loader:  # shapes: [B,K,D],[B,K],[B,C],[B,K]
            opt.zero_grad()
            log_sigma = net(feats.to(device), mask.to(device), ctx.to(device))
            loss = heteroscedastic_nll(err.to(device), log_sigma, mask=mask.to(device))
            loss.backward()
            opt.step()
        # TODO(intern): validate on split.calibration (sequence-disjoint) and log NLL/coverage.
    torch.save(net.state_dict(), out_path)


def train_net_b(h5_dir: str, out_path: str, epochs: int, lr: float, device: str) -> None:
    split = EUROC_SEQUENCE_DISJOINT_SPLIT
    ds = NetBDump(h5_dir, split.train)
    loader = DataLoader(ds, batch_size=64, shuffle=True)
    net = NetB(use_std_pool=True).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    for epoch in range(epochs):
        for imu_window, err in loader:  # [B,20,6], [B,4]
            opt.zero_grad()
            log_scales = net(imu_window.to(device))
            # Net B error target is the preintegration error per noise channel (Section 8.3).
            loss = heteroscedastic_nll(err.to(device), log_scales)
            loss.backward()
            opt.step()
    torch.save(net.state_dict(), out_path)


def main() -> None:
    ap = argparse.ArgumentParser(description="conformal Stage-2 offline head trainer")
    ap.add_argument("--head", choices=["A", "B"], required=True)
    ap.add_argument("--h5-dir", required=True, help="dir of Stage-1 <seq>.h5 dumps")
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()
    if args.head == "A":
        train_net_a(args.h5_dir, args.out, args.epochs, args.lr, args.device)
    else:
        train_net_b(args.h5_dir, args.out, args.epochs, args.lr, args.device)


if __name__ == "__main__":
    main()
