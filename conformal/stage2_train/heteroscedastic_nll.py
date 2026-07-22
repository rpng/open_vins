#!/usr/bin/env python3
"""
heteroscedastic_nll.py  --  the training loss for both heads.

Companion to: Section 8.3 ("Training: fully offline") -- the "heteroscedastic NLL" box.

THE LOSS (verbatim from the PDF):

        L = 1/2 * (e / sigma)^2  +  1/2 * log(sigma^2)

The first term punishes claiming a small sigma when the error e is large; the second
punishes inflating sigma to escape the first. Setting dL/d(sigma^2) = 0 gives sigma^2 = e^2,
so in expectation the minimiser is sigma^2 = E[e^2] -- the network is driven to output the
TRUE conditional error variance. That is exactly the quantity a calibrated R (Net A) or Q
(Net B) needs.

This is what makes the whole project tractable in weeks (Section 8.3 / Section 17): the loss
is computed on dumped residuals, so NO gradient ever flows through the filter and no
differentiable-EKF machinery is needed.

Networks emit log_sigma (unconstrained) -> we exponentiate here, so sigma > 0 by construction.
"""

from __future__ import annotations

import torch


def heteroscedastic_nll(error: torch.Tensor, log_sigma: torch.Tensor,
                        mask: torch.Tensor | None = None,
                        log_sigma_clamp: tuple[float, float] = (-7.0, 7.0)) -> torch.Tensor:
    """Mean heteroscedastic negative log-likelihood.

    Args:
        error:     e = measured error (Net A: ||z_i - h(x_gt, p_i)||; Net B: preintegration error).
        log_sigma: network output; sigma = exp(log_sigma).
        mask:      optional {0,1} mask for padded set elements (Net A variable K). Broadcastable.
        log_sigma_clamp: clamp log_sigma before exp() for numerical stability.

    Returns:
        scalar mean loss (masked mean if a mask is given).

    Note the loss in terms of s = log_sigma is:  L = 1/2 * e^2 * exp(-2s) + s   (drop const).
    """
    s = log_sigma.clamp(*log_sigma_clamp)
    per_elem = 0.5 * (error ** 2) * torch.exp(-2.0 * s) + s
    if mask is None:
        return per_elem.mean()
    denom = mask.sum().clamp_min(1.0)
    return (per_elem * mask).sum() / denom


if __name__ == "__main__":
    # Sanity: with fixed errors, the minimising sigma should approach sqrt(E[e^2]).
    torch.manual_seed(0)
    e = torch.randn(10000).abs() * 0.7            # true RMS ~ 0.7
    s = torch.zeros(1, requires_grad=True)        # single shared log_sigma
    opt = torch.optim.Adam([s], lr=0.05)
    for _ in range(2000):
        opt.zero_grad()
        loss = heteroscedastic_nll(e, s.expand_as(e))
        loss.backward()
        opt.step()
    print(f"recovered sigma = {s.exp().item():.4f}  vs  sqrt(E[e^2]) = {e.pow(2).mean().sqrt().item():.4f}")
