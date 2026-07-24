#!/usr/bin/env python3
"""
net_b_imu_dilated_tcn.py  --  Net B: IMU process-noise head.

Companion to: Section 8.2 ("Net B: why dilated convolutions and a standard-deviation pool")
              and the architecture table on p.14.

WHAT NET B DOES
---------------
Stock OpenVINS' entire "distrust of its own motion" is four scalar IMU noise densities
held constant for the whole flight (ov_msckf NoiseManager: sigma_w, sigma_a, sigma_wb,
sigma_ab -- see Propagator.h, Section 4.2 "Q is four numbers"). Net B reads the raw 20x6
IMU window and outputs FOUR log-scale corrections delta_k, applied as:

        sigma_k  <-  sigma_k * exp(delta_k)          (k in {w, a, wb, ab})

so Qc inflates when the window shows vibration/saturation and relaxes when the ride is
smooth (Section 4.2, "Net B makes exactly these four numbers input-dependent").

WHY A DILATED TCN: dilations 1,2,4 over 3 residual blocks give a receptive field ~29 > the
20-sample window, so every output sees the whole window in both directions, by construction,
with no recurrence (Section 8.2). Bi-GRU was discarded as "aesthetic, not principled"
(Section 10).

WHY THE STD POOL (this is the physically-motivated part, and A6 tests it): vibration and
clipping live in the HIGH-FREQUENCY content of the IMU signal. Mean and max destroy that --
a propeller resonance and a smooth manoeuvre can share a mean. The standard deviation across
the window is the fingerprint of a reading that cannot be safely integrated.
See conformal/experiments/ablation_a6_netb_std_pool.py.

INPUT:  raw IMU window [B, 20, 6]  (6 = gyro xyz + accel xyz)
OUTPUT: 4 log-scales   [B, 4]      -> multiply the 4 NoiseManager sigmas by exp(.)

VERIFIED PARAM COUNT: 150,468 for the executable two-convolution residual
blocks below.  The earlier 102,468 scaffold value did not match its own
implementation.

TODO(intern): confirm channel order / normalisation of the 20x6 window matches what
run_asl_msckf.cpp + DiagnosticsLogger.hpp dump, then lock the param-count test.
"""

from __future__ import annotations

import torch
import torch.nn as nn

WINDOW = 20              # IMU samples per camera frame window (Section 4.2 / p.14)
IN_CHANNELS = 6          # gyro(3) + accel(3)
CHANNELS = 64            # TCN channel width
KERNEL = 5               # stem kernel
DILATIONS = (1, 2, 4)    # receptive field ~29 > 20 (Section 8.2)
POOL_HIDDEN = 128        # Linear(192 -> 128) after [mean;max;std] pool
N_OUT = 4                # 4 IMU noise-density log-scales
TARGET_PARAM_COUNT = 150_468


class ResidualBlock(nn.Module):
    """Dilated Conv1d residual block, 64 channels, 'same' length (causal-agnostic, two-sided)."""

    def __init__(self, dilation: int) -> None:
        super().__init__()
        pad = (KERNEL - 1) // 2 * dilation
        self.conv1 = nn.Conv1d(CHANNELS, CHANNELS, KERNEL, padding=pad, dilation=dilation)
        self.conv2 = nn.Conv1d(CHANNELS, CHANNELS, KERNEL, padding=pad, dilation=dilation)
        self.act = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:  # x: [B, C, T]
        y = self.act(self.conv1(x))
        y = self.conv2(y)
        # TODO(intern): 'same' padding with even kernel/dilation can drift length by 1; crop if needed.
        return self.act(x + y[..., : x.shape[-1]])


def mean_max_std_pool(x: torch.Tensor) -> torch.Tensor:
    """Pool over time -> [mean; max; std] = 3*CHANNELS (=192). The std term is the A6 claim."""
    mean = x.mean(dim=-1)
    mx = x.max(dim=-1).values
    std = x.std(dim=-1)
    return torch.cat([mean, mx, std], dim=-1)            # [B, 3*CHANNELS]


class NetB(nn.Module):
    """Dilated TCN emitting 4 IMU noise-density log-scales from a 20x6 window."""

    def __init__(self, use_std_pool: bool = True) -> None:
        super().__init__()
        self.use_std_pool = use_std_pool  # A6 flips this off to test the std-pool claim
        self.stem = nn.Conv1d(IN_CHANNELS, CHANNELS, KERNEL, padding=(KERNEL - 1) // 2)
        self.blocks = nn.Sequential(*[ResidualBlock(d) for d in DILATIONS])
        pool_dim = 3 * CHANNELS if use_std_pool else 2 * CHANNELS
        self.head = nn.Sequential(
            nn.Linear(pool_dim, POOL_HIDDEN), nn.ReLU(),
            nn.Linear(POOL_HIDDEN, N_OUT),
        )

    def forward(self, imu_window: torch.Tensor) -> torch.Tensor:
        """imu_window: [B, WINDOW, IN_CHANNELS]  ->  log_scales: [B, 4]."""
        x = imu_window.transpose(1, 2)                   # [B, IN_CHANNELS, WINDOW]
        x = self.blocks(self.stem(x))                    # [B, CHANNELS, WINDOW]
        if self.use_std_pool:
            pooled = mean_max_std_pool(x)
        else:
            pooled = torch.cat([x.mean(dim=-1), x.max(dim=-1).values], dim=-1)  # A6 ablation
        return self.head(pooled)                         # [B, 4] log-scales


def apply_log_scales(base_sigmas: torch.Tensor, log_scales: torch.Tensor) -> torch.Tensor:
    """sigma_k <- sigma_k * exp(delta_k). base_sigmas order: [sigma_w, sigma_a, sigma_wb, sigma_ab]."""
    return base_sigmas * torch.exp(log_scales)


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    net = NetB(use_std_pool=True)
    n = count_parameters(net)
    print(f"NetB parameters: {n} (verified target {TARGET_PARAM_COUNT})")
    assert n == TARGET_PARAM_COUNT, (n, TARGET_PARAM_COUNT)
    out = net(torch.randn(4, WINDOW, IN_CHANNELS))
    print("output shape:", tuple(out.shape), "(expected (4, 4))")
