#!/usr/bin/env python3
"""
claim_c3_conformal_repair_and_threshold.py  --  C3: the repair works, and theory predicts where
                                                 the failure happens.

Companion to: Section 13.4 (claim C3). Two distinct parts; the second is the more valuable.

------------------------------------------------------------------------------------------
C3a -- THE REPAIR: conformalised Q,R restore consistency zero-shot.
PROTOCOL (Section 13.4): compute per-modality q_alpha on a SEQUENCE-DISJOINT calibration pool
(split_conformal_per_modality.py). Apply sigma_tilde = q_alpha * sigma. Evaluate on the
corruption sweep and on TUM-VI / UMA-VI zero-shot. Report BLOCK-BOOTSTRAP confidence intervals
on every coverage number (with only a handful of calibration sequences the quantile is itself
uncertain -- pretending otherwise would be the same overconfidence the paper is about).
MEASURED: nees vs severity; empirical coverage vs nominal 1-alpha; sharpness (interval width); ECE.
SUCCESS: nees returns to a band around n across the sweep; coverage lands NEAR NOMINAL from
either side (target is nominal, not maximum -- over-covering means uselessly wide intervals);
sharpness stays materially better than the stock filter's blunt fixed covariance.

------------------------------------------------------------------------------------------
C3b -- THEORY FORECASTS THE BREAK POINT: the measured overconfidence factors a_hat and
efficiency loss Gamma_hat land where Theorem 2 says they should.
WHY C3b IS THE DIFFERENCE BETWEEN A POSTER AND A SPOTLIGHT (Section 13.4): anyone can show a
method fixes a problem; showing a closed-form threshold derived from first principles predicts
IN ADVANCE the severity at which a real filter breaks means the theory is a working instrument.
PROTOCOL (Section 13.4): for each severity:
    * estimate a_hat = E[e^2]/sigma^2 per modality from residuals against ground truth;
    * estimate kappa from the relative true precisions;
    * compute the predicted crossover 2*kappa/(kappa-1)  (theory.verify_theory.crossover_threshold);
    * independently locate the empirical severity where the naive-learned arm crosses the
      discard-that-sensor baseline.
    Compare.
SUCCESS: predicted and empirical crossover coincide within the resolution of the severity grid.

IF C3b FAILS BUT C3a HOLDS (Section 13.4): the paper still stands -- demote the theory from
prediction to motivation and lead with A3. Decide EARLY (this is also a Week-4 kill gate,
Section 22), not during the write-up.

TODO(intern): C3a reuses the conformalised arm from plug_learned_sigma.py + metrics; C3b needs
the per-severity a_hat/kappa estimation from dumped residuals and the crossover comparison.
"""

from __future__ import annotations

# from theory.verify_theory import crossover_threshold
# from stage2_train.split_conformal_per_modality import fit_per_modality, Modality

SEVERITIES = (0, 1, 2, 3, 4, 5)


def run_c3a_repair(h5_dir: str, calib_sequences, test_sequences, alpha: float, out_csv: str) -> None:
    """Fit per-modality q_alpha on calib pool, evaluate conformalised arm on corruption sweep +
    TUM-VI/UMA-VI; record nees/n, coverage (+ block-bootstrap CI), sharpness, ECE."""
    raise NotImplementedError(
        "TODO(intern): fit_per_modality on sequence-disjoint calib; evaluate; block-bootstrap CIs.")


def run_c3b_threshold(h5_dir: str, out_csv: str) -> None:
    """Per severity estimate a_hat and kappa; predicted crossover = 2*kappa/(kappa-1); locate the
    empirical crossover; compare."""
    raise NotImplementedError(
        "TODO(intern): a_hat = E[e^2]/sigma^2 per modality; kappa from true precisions; compare "
        "predicted crossover_threshold(kappa) to the empirical naive-vs-discard crossover severity.")


def check_c3a_success(results) -> bool:
    """nees/n in a band around 1 and coverage near nominal 1-alpha across the sweep."""
    raise NotImplementedError("TODO(intern): encode the C3a band + near-nominal coverage test")


def check_c3b_success(predicted_severity: float, empirical_severity: float, grid_step: float = 1.0) -> bool:
    """Predicted and empirical crossover coincide within one severity step."""
    ok = abs(predicted_severity - empirical_severity) <= grid_step
    print(f"  predicted crossover severity = {predicted_severity:.2f}, empirical = "
          f"{empirical_severity:.2f}, within grid step {grid_step}: {ok}")
    return ok


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
