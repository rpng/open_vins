#!/usr/bin/env python3
"""
ablation_a3_uniform_miscalibration.py  --  A3: the cleanest experiment in the project.

Companion to: Section 14.1 ("A3 in detail -- the cleanest experiment in the project") and the
              Proposition (Section 6.6). Scheduled for Week 3 (Section 21); needs NO networks.

DESIGN (Section 14.1): take stock, well-tuned OpenVINS. Multiply BOTH Q and R by a common
scalar lambda in {0.1, 0.25, 0.5, 1, 2, 4}. No learning anywhere.

PREDICTION FROM COROLLARY 1.1 (Section 14.1):
    ATE is EXACTLY unchanged; nees scales as 1/lambda.
Why exactly: the Kalman gain is invariant under (P -> lambda P, R -> lambda R):
    K = lambda P H^T (lambda H P H^T + lambda R)^-1 = P H^T (H P H^T + R)^-1,
so the entire state trajectory is bit-identical (ATE unchanged) while P is scaled by lambda
throughout (nees scales by 1/lambda). A reviewer can verify it mentally.

WHY IT EARNS MAIN-PAPER SPACE (Section 14.1): it is the first EXPERIMENTAL separation of the
two failure modes -- a system can be arbitrarily overconfident while every trajectory-error
number stays pristine. If the manuscript has one figure that survives hostile reading, this is it.
It is also observability-independent, so it doubles as an FEJ-confound control (Section 15).

THE TECHNICAL TRAP (Section 14.1, "A technical trap in A3") -- DO NOT SKIP:
    The gain invariance holds for the state update, but OpenVINS gates outliers with a
    Mahalanobis chi-squared test that READS P and is NOT scale-invariant. Scaling lambda
    silently changes which measurements are accepted, breaking the exact-ATE invariance and
    muddying the result. Either:
        (a) scale the gate threshold with lambda, or
        (b) disable the gate for this ablation,
    and STATE which you did in the paper. Concretely, in config/euroc_mav/estimator_config.yaml
    the knobs are `up_msckf_chi2_multipler` (and up_slam_/up_aruco_). Option (a): multiply the
    chi2 multiplier by lambda when you multiply Q,R. Getting this wrong turns the project's
    cleanest experiment into a confusing one.

SUCCESS CRITERION: across lambda, ATE is invariant to within numerical noise AND nees/n tracks
1/lambda (a straight line of slope -1 on a log-log plot). Deviation localises either a residual
scale-dependence or the chi2-gate trap above.

NOTE: A3 is pure config manipulation -- it does NOT need Net A, Net B, HDF5 dumps, or conformal
machinery. It only needs a working Stage-1/Gate-2 OpenVINS build. That is why Week 3 can run it
even if training is still rough (Section 21).
"""

from __future__ import annotations

LAMBDAS = (0.1, 0.25, 0.5, 1.0, 2.0, 4.0)


def scaled_config(base_config_yaml: str, lam: float, scale_chi2_gate: bool, out_yaml: str) -> None:
    """Write a copy of the EuRoC config with Q and R (and optionally the chi2 gate) scaled by lam.

    Q lives in the IMU noise densities (NoiseManager: gyroscope/accelerometer *_noise_density and
    *_random_walk in the kalibr_imu_chain.yaml); R lives in up_msckf_sigma_px (and up_slam_/up_aruco_).
    Scaling *variance* by lambda means scaling each SIGMA by sqrt(lambda). If scale_chi2_gate,
    multiply up_*_chi2_multipler by lambda (option (a) of the trap).

    TODO(intern): load the YAML, apply the scalings, dump out_yaml. Prefer editing sigmas over
    patching code so ov_msckf stays untouched.
    """
    raise NotImplementedError("TODO(intern): scale sigmas by sqrt(lambda); optionally scale chi2 gate")


def run_a3(base_config_yaml: str, seq_dir: str, out_csv: str, scale_chi2_gate: bool = True) -> None:
    """For each lambda: write a scaled config, run OpenVINS, record ATE and nees/n."""
    raise NotImplementedError(
        "TODO(intern): loop LAMBDAS -> scaled_config -> run_asl_msckf -> metrics; append (lambda, ate, "
        "nees_over_n) rows to out_csv.")


def check_success(rows) -> bool:
    """ATE ~ const across lambda; nees_over_n ~ C/lambda (log-log slope ~ -1)."""
    import numpy as np
    lam = np.array([r["lambda"] for r in rows], float)
    ate = np.array([r["ate"] for r in rows], float)
    nees = np.array([r["nees_over_n"] for r in rows], float)
    ate_ok = ate.std() / (ate.mean() + 1e-12) < 0.02          # ATE invariant to ~2%
    slope = np.polyfit(np.log(lam), np.log(nees), 1)[0]         # expect ~ -1
    slope_ok = abs(slope + 1.0) < 0.1
    print(f"  ATE coeff-of-variation = {ate.std()/(ate.mean()+1e-12):.4f} (want <0.02)")
    print(f"  log-log nees vs lambda slope = {slope:.3f} (want ~ -1.0)")
    return bool(ate_ok and slope_ok)


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
