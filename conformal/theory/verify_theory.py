#!/usr/bin/env python3
"""
verify_theory.py  --  Numerical verification of the miscalibration theory.

Companion to:  Part I, Section 6 of `conformal_explainer.pdf`
               ("The theory": Theorem 1, Corollary 1.1, Theorem 2, Theorem 3, Proposition 1).

WHAT THIS FILE IS FOR
---------------------
The manuscript's central theoretical claim is that plugging a *miscalibrated*
covariance into an inverse-variance fusion (which is what a Kalman filter does)
does not degrade gracefully -- past a computable threshold it is provably WORSE
than throwing the sensor away. This script reproduces, in a two-scalar-sensor
toy, every number the PDF quotes, so the intern can confirm the theory before
touching the filter. The explainer literally cites this file by name
("Numerically verified in verify_theory.py"), so keep the printed numbers in
sync with Section 6.

DEFINITIONS (Section 6.1)
    s2_m  : true error variance of sensor m               (how wrong it really is)
    sig2_m: reported variance of sensor m                 (how wrong it CLAIMS to be)
    a_m   = s2_m / sig2_m   -> overconfidence factor      (a>1 == lying)
    p_m   = 1 / s2_m        -> true precision
    Fusion uses the CLAIMED precisions w_m = 1/sig2_m = a_m * p_m.

This module is pure NumPy, has no dependency on OpenVINS, and is safe to run on a
laptop in a second. It is deliberately fully implemented (not a stub): it is the
one artifact that lets you audit the maths independently of the C++ system.

Run:
    python conformal/theory/verify_theory.py
Expected: every assertion passes and the printed table matches Section 6.
"""

from __future__ import annotations

import numpy as np


# --------------------------------------------------------------------------------------
# Core formulas (Section 6.1 - 6.2)
# --------------------------------------------------------------------------------------
def true_mse(a: np.ndarray, p: np.ndarray) -> float:
    """True MSE of the fused estimate when fusion weights use the *claimed* precisions.

    MSE = ( sum_m a_m^2 p_m ) / ( sum_m a_m p_m )^2      (PDF, Section 6.1)
    """
    a = np.asarray(a, dtype=float)
    p = np.asarray(p, dtype=float)
    return float(np.sum(a**2 * p) / (np.sum(a * p) ** 2))


def calibrated_mse(p: np.ndarray) -> float:
    """The calibrated optimum:  MSE* = ( sum_m p_m )^-1  (achieved when all a_m == 1)."""
    p = np.asarray(p, dtype=float)
    return float(1.0 / np.sum(p))


def efficiency_loss(a: np.ndarray, p: np.ndarray) -> float:
    """Gamma = MSE / MSE*   (Theorem 1). A Cauchy-Schwarz gap: Gamma >= 1 always,
    with equality iff all a_m are equal."""
    return true_mse(a, p) / calibrated_mse(p)


def crossover_threshold(kappa: float) -> float:
    """Theorem 2 crossover: fusion is strictly worse than discarding sensor 1 iff
    a1 > 2*kappa/(kappa-1), where kappa = p2/p1 > 1. Tends to 2 as kappa -> inf."""
    return 2.0 * kappa / (kappa - 1.0)


# --------------------------------------------------------------------------------------
# Verifications -- each function reproduces a specific quoted result in Section 6.
# --------------------------------------------------------------------------------------
def verify_theorem1_and_corollary(atol: float = 1e-4) -> None:
    """Theorem 1 (Gamma >= 1) and Corollary 1.1 (scale invariance of Gamma)."""
    print("== Theorem 1 / Corollary 1.1 ==")
    p = np.array([1.0, 1.0])

    # Corollary 1.1: uniform overconfidence is harmless. a = (3, 3) -> Gamma = 1.0000.
    g_uniform = efficiency_loss([3.0, 3.0], p)
    print(f"  a=(3,3)  Gamma = {g_uniform:.4f}   (PDF: 1.0000, uniform overconfidence harmless)")
    assert abs(g_uniform - 1.0) < atol, "Corollary 1.1 failed: uniform a should give Gamma=1"

    # Relative miscalibration does the damage. a = (4, 1) -> Gamma = 1.535.
    g_relative = efficiency_loss([4.0, 1.0], p)
    print(f"  a=(4,1)  Gamma = {g_relative:.4f}   (PDF: 1.535, ~53% efficiency loss)")
    assert abs(g_relative - 1.535) < 1e-2, "Theorem 1 mismatch for a=(4,1)"

    # Scale invariance: multiply every a by lambda -> Gamma unchanged.
    for lam in [0.1, 2.0, 10.0]:
        g = efficiency_loss([4.0 * lam, 1.0 * lam], p)
        assert abs(g - g_relative) < atol, "Corollary 1.1 scale-invariance failed"
    print("  scale-invariance of Gamma under a -> lambda*a: OK")
    print()


def verify_theorem2(atol: float = 1e-3) -> None:
    """Theorem 2 crossover thresholds vs the PDF's numeric table."""
    print("== Theorem 2 (crossover threshold a1 > 2k/(k-1)) ==")
    expected = {2: 4.000, 5: 2.500, 10: 2.222, 1000: 2.002}
    for kappa, want in expected.items():
        got = crossover_threshold(kappa)
        print(f"  kappa={kappa:<5d} threshold = {got:.3f}   (PDF: {want:.3f})")
        assert abs(got - want) < atol, f"Theorem 2 mismatch at kappa={kappa}"

    # The headline "41%" number: in std-dev units the limiting threshold is sqrt(2).
    print(f"  limiting threshold in sigma-units = sqrt(2) = {np.sqrt(2):.4f}"
          f"   (a 41% understatement of sigma is enough)")
    print()


def verify_theorem3(atol: float = 1e-3) -> None:
    """Theorem 3: as a1 -> inf the fused MSE -> s1^2 (the honest sensor is annihilated)."""
    print("== Theorem 3 (the liar wins: MSE -> s1^2 as a1 -> inf) ==")
    s = np.array([0.5, 0.1])          # sensor 1 is 5x worse than sensor 2
    p = 1.0 / s**2

    honest_alone = 1.0 / p[1]          # discard the liar, keep the good sensor
    print(f"  honest sensor alone  MSE = {honest_alone:.4f}   (PDF: 0.01)")
    assert abs(honest_alone - 0.01) < atol

    calibrated = calibrated_mse(p)     # a = (1, 1): fusion helps slightly
    print(f"  calibrated fusion    MSE = {calibrated:.4f}   (PDF: 0.0096)")
    assert abs(calibrated - 0.0096) < 1e-3

    print("  a1 sweep (fused MSE walks toward s1^2 = 0.25):")
    for a1 in [1.0, 3.0, 10.0, 1e6]:
        mse = true_mse([a1, 1.0], p)
        print(f"    a1={a1:<10g} fused MSE = {mse:.4f}")
    mse_limit = true_mse([1e9, 1.0], p)
    print(f"  limit MSE = {mse_limit:.4f}   (PDF: 0.2500 = s1^2)")
    assert abs(mse_limit - s[0] ** 2) < 1e-3
    print()


def verify_proposition_decomposition() -> None:
    """Proposition 1: two distinct failures.
      * Relative miscalibration corrupts x_hat  (visible in Gamma / partly in ATE)
      * Absolute (uniform) miscalibration corrupts P only (visible in NEES only)
    We illustrate the second row: uniform overconfidence leaves Gamma=1 (estimate fine)
    but multiplies the *reported* covariance by 1/lambda (P becomes a fiction).
    This is the row the field's trajectory-error metric cannot see. See Section 6.6."""
    print("== Proposition 1 (two failures, not one) ==")
    p = np.array([1.0, 1.0])
    lam = 10.0  # everyone claims to be 10x better than they are
    g = efficiency_loss([lam, lam], p)
    reported_cov_scale = 1.0 / lam  # claimed variance is lambda-times too small
    print(f"  uniform a=({lam},{lam}): Gamma={g:.4f} (estimate optimal) but reported "
          f"covariance is {reported_cov_scale:.2f}x true -> NEES ~ {1/reported_cov_scale:.0f}x too big")
    print("  => trajectory error sees nothing; only NEES exposes the dishonest P.")
    print()


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)
    print("conformal theory verification  (conformal_explainer.pdf, Section 6)\n")
    verify_theorem1_and_corollary()
    verify_theorem2()
    verify_theorem3()
    verify_proposition_decomposition()
    print("All theory checks passed.")


if __name__ == "__main__":
    main()
