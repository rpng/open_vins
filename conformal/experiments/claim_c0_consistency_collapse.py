#!/usr/bin/env python3
"""
claim_c0_consistency_collapse.py  --  C0: naive learned covariance destroys consistency
                                       while accuracy looks fine.

Companion to: Section 13.1 (claim C0). THIS CLAIM IS THE PAPER (Section 13.1, "Why C0 exists"):
the theory predicts amplification; C0 demonstrates the predicted pathology in a real,
unmodified, widely-used filter -- not just a two-sensor toy. Nothing else in the manuscript
survives its failure. This is also the Week-2 de-risk spike (Section 21) and a KILL GATE
(Section 22: "nees does not visibly break at end of Week 2 -> Pivot").

STATEMENT: under zero-shot degradation, plugging naive learned Q,R into OpenVINS makes NEES
explode while ATE stays plausible -- i.e. the failure is invisible to the metric the field reports.

PROTOCOL (Section 13.1):
    1. Train Nets A,B on clean EuRoC with the test sequences held out (sequence-disjoint).
    2. Apply the corruption suite at severities 0..5 to the held-out sequences.
    3. Run three arms over the full sweep:
         (i)  stock   -- unmodified OpenVINS, default Q,R
         (ii) learned -- Nets A,B plugged in, NO conformal
         (iii)oracle  -- sigma from ground-truth errors (the achievable floor)
    4. Cross every arm with FEJ on / FEJ off (Section 15; see the_fej_confound_control.py).
    5. Repeat across seeds; report medians with IQRs (single-run VIO numbers are noisy).

MEASURED: ATE, RTE per (severity, arm); nees/n per (severity, arm); empirical nees vs its
chi2_n reference.

SUCCESS CRITERION (Section 13.1): a severity range exists where the ATE ratio (ii):(i) is
below ~1.2 WHILE the nees ratio exceeds ~5. That gap is the result.

IF IT FAILS (Section 13.1): there is no paper. Pivot immediately; do not spend a month
searching for the effect.

TODO(intern): wire the arm runner (plug_learned_sigma.py -> Stage-3 run -> metrics), the
severity/seed loops, and aggregation into the results CSV consumed by make_money_plot.py.
"""

from __future__ import annotations

from dataclasses import dataclass

# from stage3_eval.metrics_nees_ate_rte import nees_over_n, ate_rte_via_ov_eval
# from stage3_eval.plug_learned_sigma import Arm, build_sigma_sidecar

SEVERITIES = (0, 1, 2, 3, 4, 5)
SEEDS = (0, 1, 2, 3, 4)
ARMS = ("stock", "learned", "oracle")
ATE_RATIO_MAX = 1.2   # ATE(ii)/ATE(i) must stay below this...
NEES_RATIO_MIN = 5.0  # ...while nees(ii)/nees(i) exceeds this. The gap is the result.


@dataclass
class ArmSeverityResult:
    arm: str
    severity: int
    seed: int
    fej: bool
    ate: float
    rte: float
    nees_over_n: float


def run_c0(h5_dir: str, out_csv: str) -> None:
    """Run the full C0 sweep and write the aggregated results CSV."""
    raise NotImplementedError(
        "TODO(intern): for each (arm, severity, seed, fej): build the sigma sidecar for the "
        "arm, run the Stage-3 OpenVINS pass on the corrupted sequence, compute nees/n + ATE/RTE, "
        "append an ArmSeverityResult row, then write out_csv.")


def check_success(results: list[ArmSeverityResult]) -> bool:
    """Encode the Section 13.1 success criterion: exists a severity with ATE ratio < 1.2 and
    nees ratio > 5 (learned vs stock), medians across seeds, FEJ ON (the hard case)."""
    import numpy as np
    ok = False
    for sev in SEVERITIES:
        def med(arm, field):
            xs = [getattr(r, field) for r in results
                  if r.arm == arm and r.severity == sev and r.fej]
            return float(np.median(xs)) if xs else float("nan")
        ate_ratio = med("learned", "ate") / max(med("stock", "ate"), 1e-9)
        nees_ratio = med("learned", "nees_over_n") / max(med("stock", "nees_over_n"), 1e-9)
        if ate_ratio < ATE_RATIO_MAX and nees_ratio > NEES_RATIO_MIN:
            print(f"  severity {sev}: ATE ratio {ate_ratio:.2f} (<{ATE_RATIO_MAX}) and "
                  f"nees ratio {nees_ratio:.1f} (>{NEES_RATIO_MIN})  <-- C0 gap")
            ok = True
    return ok


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
