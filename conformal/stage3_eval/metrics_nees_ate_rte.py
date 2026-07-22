#!/usr/bin/env python3
"""
metrics_nees_ate_rte.py  --  the metrics the paper is measured on.

Companion to: Section 7 ("How any of this gets measured") and the metric boxes therein
              (NEES, ATE/RTE, coverage/ECE/sharpness, consistency).

THE HEADLINE METRIC IS NEES (Section 7). With ground truth x, estimate x_hat, claimed
covariance P:
        epsilon = (x - x_hat)^T P^-1 (x - x_hat)
If the filter is honest, epsilon ~ chi-squared with n dof, so E[epsilon] = n (the state
dimension, n = 15 + 6*N_clones ~ 81 by default; Section 4.1).
    epsilon ~= n : consistent / honest.
    epsilon >> n : OVERCONFIDENT -- P is a fiction. THIS is the dangerous direction and the
                   whole point of the paper (a planner reading this P cuts corners it can't afford).
    epsilon << n : conservative -- wasteful but safe.
NEES needs ground truth, so it is computable only with motion capture -- part of why the
field under-reports it (Section 7). Report nees/n so it is comparable across n.

ATE / RTE (Section 7) are PURE ACCURACY metrics ("where did it think it was?") and are
STRUCTURALLY BLIND to a covariance that lies -- that blindness is the paper's thesis
(Proposition 1, Section 6.6). OpenVINS already ships a mature aligner in ov_eval; prefer it
(see below) instead of re-deriving Umeyama alignment here.

COVERAGE / ECE / SHARPNESS (Section 7) quantify calibration:
    coverage  : fraction of true errors inside the claimed interval; target = NOMINAL 1-alpha,
                not maximum (over-covering means uselessly wide intervals).
    ECE       : average gap between promised and delivered confidence across levels.
    sharpness : how narrow the intervals are (report WITH coverage -- either alone is gameable).
"""

from __future__ import annotations

import numpy as np


def nees(error: np.ndarray, P: np.ndarray) -> float:
    """Single-sample NEES: e^T P^-1 e. `error` is the state error vector (x - x_hat) in the
    filter's error-state parameterisation (use delta-theta for orientation, NOT raw quaternion
    difference -- Section 4.1). P must be in the same ordering."""
    error = np.asarray(error, dtype=float)
    P = np.asarray(P, dtype=float)
    return float(error @ np.linalg.solve(P, error))


def average_nees(errors: np.ndarray, covariances: np.ndarray) -> float:
    """Mean NEES over a trajectory. errors: [T, d]; covariances: [T, d, d]."""
    vals = [nees(e, P) for e, P in zip(errors, covariances)]
    return float(np.mean(vals))


def nees_over_n(errors: np.ndarray, covariances: np.ndarray, n: int) -> float:
    """average_nees / n. ~1.0 is consistent; >>1 is the overconfident failure the paper hunts."""
    return average_nees(errors, covariances) / n


def chi2_consistency_bounds(n: int, num_samples: int, alpha: float = 0.05) -> tuple[float, float]:
    """Two-sided chi-squared consistency interval for the AVERAGE nees/n over num_samples runs.
    Points outside [lo, hi] are statistically inconsistent. Used to draw the acceptance band in
    the money plot (Section 16)."""
    from scipy.stats import chi2  # TODO(intern): scipy is only needed here; keep import local.
    dof = n * num_samples
    lo = chi2.ppf(alpha / 2, dof) / dof
    hi = chi2.ppf(1 - alpha / 2, dof) / dof
    return float(lo), float(hi)


def coverage(errors: np.ndarray, sigmas: np.ndarray) -> float:
    """Empirical coverage: fraction with |e| <= sigma (sigma already conformally rescaled).
    Compare to the nominal 1 - alpha; landing near nominal from EITHER side is the target."""
    errors = np.asarray(errors, dtype=float)
    sigmas = np.asarray(sigmas, dtype=float)
    return float(np.mean(np.abs(errors) <= sigmas))


def expected_calibration_error(errors: np.ndarray, sigmas: np.ndarray,
                               levels: np.ndarray | None = None) -> float:
    """ECE for a Gaussian error model: average |empirical - nominal| coverage across levels.
    For each nominal p, the interval half-width is z_p * sigma; measure realised coverage."""
    from scipy.stats import norm
    errors = np.asarray(errors, dtype=float)
    sigmas = np.asarray(sigmas, dtype=float)
    if levels is None:
        levels = np.linspace(0.05, 0.95, 19)
    gaps = []
    for p in levels:
        z = norm.ppf(0.5 + p / 2.0)                     # two-sided interval for coverage p
        realised = np.mean(np.abs(errors) <= z * sigmas)
        gaps.append(abs(realised - p))
    return float(np.mean(gaps))


def sharpness(sigmas: np.ndarray) -> float:
    """Mean interval width proxy = mean sigma. Report alongside coverage, never alone."""
    return float(np.mean(np.asarray(sigmas, dtype=float)))


def ate_rte_via_ov_eval(est_tum: str, gt_tum: str) -> dict:
    """ATE/RTE via OpenVINS' own evaluator (mature SE(3) alignment; do not re-derive).

    OpenVINS ships ov_eval with `ov_eval error_comparison` / `error_dataset` and the repo's
    benchmark/ scripts already convert trajectories to TUM and compute APE/RPE. Reuse that:
      - benchmark/trajectory_to_tum.py  ->  TUM files
      - ov_eval (or evo) ->  ATE (== APE) and RTE (== RPE)
    TODO(intern): shell out to ov_eval or `evo_ape/evo_rpe`, parse, and return the numbers.
    """
    raise NotImplementedError("TODO(intern): call ov_eval / evo on the TUM files and parse ATE/RTE")
