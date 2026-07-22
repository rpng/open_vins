#!/usr/bin/env python3
"""
net_a_visual_deepsets.py  --  Net A: per-feature visual measurement-noise head.

Companion to: Section 8.1 ("Net A: why a set network") and the architecture table on p.14.

WHAT NET A DOES
---------------
Stock OpenVINS uses ONE pixel-noise scalar sigma_pix for every visual feature, forever
(see ov_msckf UpdaterOptions::sigma_pix, used in UpdaterMSCKF.cpp to build R). Net A
replaces it with a PER-FEATURE sigma_pix_i, so R becomes genuinely heteroscedastic:
a crisp corner in good light earns a small sigma; a smeared blob on a dark wall earns a
large one (Section 4.3, "R is one number" -> per-feature).

WHY A SET NETWORK (DeepSets, permutation-invariant): a frame has a varying number of
tracked features (~150 in good light, ~12 in near-dark) in arbitrary order. Feature
identity is meaningless, but the DISTRIBUTION of track quality across the frame is the
signal. So: encode each feature independently -> pool (mean/max/std) -> per-feature head
that also sees the pooled summary and the frame-level context.

INPUTS (per feature, Section 8.1 + p.14 table):
    KLT forward-backward error, Shi-Tomasi corner response, track age, parallax,
    pixel (u,v), stereo left-right consistency  -> PER_FEATURE_DIM channels
FRAME-LEVEL CONTEXT (concatenated onto each feature before the head):
    num tracked, num lost, RANSAC inlier ratio, brightness (+ ...) -> FRAME_CTX_DIM
OUTPUT: one log-sigma per feature (exp -> sigma_pix_i, always positive).

TARGET PARAM COUNT: 26,305 (p.14). A unit test asserts this so architecture drift is caught.

TODO(intern): reconcile PER_FEATURE_DIM / FRAME_CTX_DIM with the exact columns you dump in
DiagnosticsLogger.hpp so the count lands on 26,305, then remove the xfail on the test below.
"""

from __future__ import annotations

import torch
import torch.nn as nn

# Dimensions from the p.14 table. Encoder 8->64->64 ; pool [mean;max;std]=192->64 ;
# head (64 + 64 + 6)->64->1. Keep these as named constants so the param count is auditable.
PER_FEATURE_DIM = 8      # per-feature raw inputs (KLT fbe, corner resp, age, parallax, u, v, stereo, ...)
ENC_HIDDEN = 64
ENC_OUT = 64
FRAME_CTX_DIM = 6        # frame-level context appended before the head
POOL_OUT = 64            # Linear(192 -> 64) frame summary
HEAD_HIDDEN = 64
TARGET_PARAM_COUNT = 26_305


class PerFeatureEncoder(nn.Module):
    """8 -> 64 -> 64 shared MLP applied independently to every feature (the DeepSets 'phi')."""

    def __init__(self, in_dim: int = PER_FEATURE_DIM) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, ENC_HIDDEN), nn.ReLU(),
            nn.Linear(ENC_HIDDEN, ENC_OUT), nn.ReLU(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:  # x: [B, K, PER_FEATURE_DIM]
        return self.net(x)                               # -> [B, K, ENC_OUT]


def masked_mean_max_std(enc: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    """Order-blind pool over the K features -> [mean; max; std] = 3*ENC_OUT (=192).

    mask: [B, K] with 1 for real features, 0 for padding (frames have variable K).
    TODO(intern): make the masked std numerically safe when a frame has <2 features.
    """
    m = mask.unsqueeze(-1)                               # [B, K, 1]
    n = m.sum(dim=1).clamp_min(1.0)                      # [B, 1]
    mean = (enc * m).sum(dim=1) / n
    neg_inf = torch.finfo(enc.dtype).min
    mx = (enc.masked_fill(m == 0, neg_inf)).max(dim=1).values
    var = ((enc - mean.unsqueeze(1)) ** 2 * m).sum(dim=1) / n
    std = var.clamp_min(1e-12).sqrt()
    return torch.cat([mean, mx, std], dim=-1)            # [B, 3*ENC_OUT]


class NetA(nn.Module):
    """DeepSets head emitting one log-sigma_pix per tracked feature."""

    def __init__(self) -> None:
        super().__init__()
        self.encoder = PerFeatureEncoder(PER_FEATURE_DIM)
        self.pool_proj = nn.Linear(3 * ENC_OUT, POOL_OUT)  # 192 -> 64 frame summary
        self.head = nn.Sequential(
            nn.Linear(ENC_OUT + POOL_OUT + FRAME_CTX_DIM, HEAD_HIDDEN), nn.ReLU(),
            nn.Linear(HEAD_HIDDEN, 1),
        )

    def forward(self, feats: torch.Tensor, mask: torch.Tensor, frame_ctx: torch.Tensor) -> torch.Tensor:
        """feats: [B,K,PER_FEATURE_DIM]  mask: [B,K]  frame_ctx: [B,FRAME_CTX_DIM]
        returns log_sigma: [B, K] (exp() it to get sigma_pix_i; clamp before exp in training)."""
        enc = self.encoder(feats)                         # [B, K, ENC_OUT]
        summary = self.pool_proj(masked_mean_max_std(enc, mask))  # [B, POOL_OUT]
        K = enc.shape[1]
        summary_b = summary.unsqueeze(1).expand(-1, K, -1)
        ctx_b = frame_ctx.unsqueeze(1).expand(-1, K, -1)
        h = torch.cat([enc, summary_b, ctx_b], dim=-1)    # [B, K, ENC_OUT+POOL_OUT+FRAME_CTX_DIM]
        return self.head(h).squeeze(-1)                   # [B, K]


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    net = NetA()
    n = count_parameters(net)
    print(f"NetA parameters: {n} (target {TARGET_PARAM_COUNT} from p.14)")
    # Smoke test forward pass with a variable-K frame.
    B, K = 2, 150
    out = net(torch.randn(B, K, PER_FEATURE_DIM), torch.ones(B, K), torch.randn(B, FRAME_CTX_DIM))
    print("output shape:", tuple(out.shape), "(expected (2, 150))")
