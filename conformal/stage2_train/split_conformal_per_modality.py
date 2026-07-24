#!/usr/bin/env python3
"""
split_conformal_per_modality.py  --  the conformal repair.

Companion to: Section 9 ("Conformal prediction: the repair") and Section 9.1
              ("Non-exchangeability, which is a contribution and not a caveat").

WHY CONFORMAL (Section 9): the theory says the damage comes from a miscalibrated sigma. The
repair must fix calibration WITHOUT assuming the network is calibrated -- which rules out most
recalibration methods. Split conformal prediction is distribution-free: it turns any model's
sigma into intervals with a guaranteed coverage rate.

SPLIT CONFORMAL (Section 9):
    1. Hold out a calibration set the model never trained on.
    2. For each calibration point compute the standardised residual   |e_i| / sigma_i.
    3. Take the empirical (1 - alpha) quantile of those -> q_alpha.
    4. Rescale every future prediction:   sigma_tilde = q_alpha * sigma.
    Guarantee:  P(|e| <= sigma_tilde) >= 1 - alpha, whatever the model is.

PER-MODALITY IS THE KEY CHOICE (Section 9): compute ONE q_alpha for the visual modality and a
SEPARATE one for the inertial modality. A per-modality correction fixes RELATIVE miscalibration
between camera and IMU -- which is precisely what Theorem 1 says corrupts the estimate x_hat.
And restoring absolute coverage fixes NEES. So one mechanism repairs BOTH failure modes of the
Proposition (Section 9, "One mechanism, both failures"), because the decomposition says they
share a cause.

NON-EXCHANGEABILITY (Section 9.1) -- trajectory data break the i.i.d. assumption twice:
    (1) temporal correlation: consecutive frames are near-duplicates.
    (2) distribution shift: the degradation test set is deliberately off-distribution.
Fixes (part of the contribution, not fine print):
    * sequence-disjoint calibration/test splits (see hdf5_dump_dataset.py).
    * group-conditional / adaptive conformal (Gibbs & Candes 2021; Barber et al. 2023;
      Tibshirani et al. 2019) for shifted/dependent data -- see adaptive_qalpha() stub.
LIMITATION (stated honestly, Section 9.1): coverage under shift holds only up to the
total-variation distance between calibration and test. Distribution-free != shift-free.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


class Modality(str, Enum):
    VISUAL = "visual"      # Net A -> sigma_pix per feature
    INERTIAL = "inertial"  # Net B -> 4 IMU noise densities


@dataclass
class ConformalScaler:
    """Holds one q_alpha per modality; applies sigma_tilde = q_alpha * sigma at inference."""
    q_alpha: dict[Modality, float]
    alpha: float

    def rescale(self, sigma: np.ndarray, modality: Modality) -> np.ndarray:
        return self.q_alpha[modality] * np.asarray(sigma)


def split_conformal_quantile(errors: np.ndarray, sigmas: np.ndarray, alpha: float) -> float:
    """Empirical (1 - alpha) quantile of the standardised residuals |e|/sigma, with the
    finite-sample correction ceil((n+1)(1-alpha))/n that gives the exchangeable coverage
    guarantee.

    Args:
        errors: |e_i| on the calibration set (from motion-capture GT).
        sigmas: sigma_i the network predicted for those same points.
        alpha:  target miscoverage (e.g. 0.1 for 90% coverage).
    """
    errors = np.asarray(errors, dtype=float)
    sigmas = np.asarray(sigmas, dtype=float)
    scores = np.abs(errors) / np.maximum(sigmas, 1e-12)
    n = scores.size
    # Finite-sample conformal level (Section 9): quantile rank (n+1)(1-alpha)/n.
    level = np.ceil((n + 1) * (1.0 - alpha)) / n
    level = min(level, 1.0)
    try:
        return float(np.quantile(scores, level, method="higher"))
    except TypeError:
        # NumPy <1.22 used the old keyword. The estimator is identical.
        return float(np.quantile(scores, level, interpolation="higher"))


def fit_per_modality(calib_errors: dict[Modality, np.ndarray],
                     calib_sigmas: dict[Modality, np.ndarray],
                     alpha: float = 0.1) -> ConformalScaler:
    """Fit one q_alpha per modality on a SEQUENCE-DISJOINT calibration pool (Section 9.1)."""
    q = {m: split_conformal_quantile(calib_errors[m], calib_sigmas[m], alpha)
         for m in calib_errors}
    return ConformalScaler(q_alpha=q, alpha=alpha)


def block_bootstrap_qalpha_ci(errors: np.ndarray, sigmas: np.ndarray, alpha: float,
                              block_len: int = 50, n_boot: int = 1000,
                              seed: int = 0) -> tuple[float, float]:
    """Block-bootstrap CI on q_alpha itself (Section 13.4 / risk table).

    With only a handful of calibration SEQUENCES the quantile estimate is itself uncertain,
    and "pretending otherwise would be the same overconfidence the paper is about." Blocks
    (not single frames) respect temporal correlation.

    Uses a circular moving-block bootstrap so every observation can start a
    block and each replicate has the original sample count.
    """
    errors = np.asarray(errors, dtype=float).reshape(-1)
    sigmas = np.asarray(sigmas, dtype=float).reshape(-1)
    if errors.shape != sigmas.shape or errors.size == 0:
        raise ValueError("errors and sigmas must be non-empty arrays of equal size")
    if not (0.0 < alpha < 1.0):
        raise ValueError("alpha must lie strictly between zero and one")
    if block_len < 1 or n_boot < 1:
        raise ValueError("block_len and n_boot must be positive")
    scores = np.abs(errors) / np.maximum(sigmas, 1e-12)
    n = scores.size
    block_len = min(int(block_len), n)
    blocks_per_sample = int(np.ceil(n / block_len))
    offsets = np.arange(block_len)
    rng = np.random.default_rng(seed)
    quantiles = np.empty(n_boot, dtype=float)
    for sample in range(n_boot):
        starts = rng.integers(0, n, size=blocks_per_sample)
        indices = ((starts[:, None] + offsets[None, :]) % n).reshape(-1)[:n]
        bootstrap_scores = scores[indices]
        quantiles[sample] = split_conformal_quantile(
            bootstrap_scores, np.ones_like(bootstrap_scores), alpha
        )
    return tuple(float(value) for value in np.quantile(quantiles, (0.025, 0.975)))


def adaptive_qalpha(*args, **kwargs) -> float:
    """Group-conditional / adaptive conformal for shifted, dependent data (Section 9.1).
    TODO(intern): implement Gibbs & Candes (2021)-style online update, or Barber et al. (2023)
    weighting, for the corruption sweep where calibration != test distribution."""
    raise NotImplementedError("TODO(intern): adaptive/group-conditional conformal")


if __name__ == "__main__":
    # Demo: an overconfident model (predicts sigma 3x too small) gets repaired to ~nominal.
    rng = np.random.default_rng(0)
    true_sigma = 0.5
    e = np.abs(rng.normal(0, true_sigma, size=5000))
    reported = np.full_like(e, true_sigma / 3.0)  # 3x overconfident
    q = split_conformal_quantile(e, reported, alpha=0.1)
    covered = np.mean(e <= q * reported)
    print(f"q_alpha = {q:.3f}  (expect ~3x to undo the 3x overconfidence)")
    print(f"empirical coverage after rescale = {covered:.3f}  (target >= 0.90)")
