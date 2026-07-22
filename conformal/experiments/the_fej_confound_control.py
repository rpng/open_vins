#!/usr/bin/env python3
"""
the_fej_confound_control.py  --  the FEJ confound and its control arm.

Companion to: Section 15 ("The FEJ confound and its control arm"). This is the most important
experimental-design point added after a literature review, and it MUST be in the annual-review
presentation because it is the objection a knowledgeable examiner will raise.

THE OBJECTION (Section 15): the VIO literature already knows EKF-based VIO is inconsistent and
underestimates uncertainty -- an entire subfield (observability-constrained EKF, FEJ, invariant
filtering) exists to address it. A reviewer will say: "Your nees explosion is the well-known
linearisation-induced one. You have not found a new failure mode; you amplified an old one."

BACKGROUND (Section 15): some state directions are unobservable (VIO cannot observe absolute
position or yaw about gravity). An EKF linearises around a CHANGING estimate and can acquire
SPURIOUS observability, so P shrinks illegitimately. First-Estimates Jacobians (FEJ) is the
standard fix: evaluate Jacobians at the first available estimate. In OpenVINS this is the
`use_fej` config key (config/euroc_mav/estimator_config.yaml: use_fej: true).

THE CONTROL (Section 15): run EVERY corruption sweep with FEJ ON and FEJ OFF, PLUS a simulation
arm using EXACT ANALYTIC Jacobians where observability-induced inconsistency is eliminated by
construction. If the nees explosion PERSISTS with FEJ enabled and in the exact-Jacobian
simulation, it cannot be attributed to linearisation -- it must come from the covariance the
networks supplied. Additionally, A3 (ablation_a3_uniform_miscalibration.py) separates the
mechanisms directly: injected uniform miscalibration has nothing to do with observability, so a
nees response there is unambiguously covariance-driven.

THIS MODULE provides the FEJ on/off crossing used by every claim driver (C0 explicitly crosses
every arm with FEJ on/off, Section 13.1 step 4).

TODO(intern): FEJ on/off is a config toggle (use_fej). The exact-Jacobian simulation arm uses
OpenVINS' simulator (ov_msckf run_simulation.cpp / config/rpng_sim) where ground-truth Jacobians
are available; wire it as a third setting here.
"""

from __future__ import annotations

FEJ_SETTINGS = ("fej_on", "fej_off", "exact_jacobian_sim")


def fej_variants_of_config(base_config_yaml: str, out_dir: str) -> dict:
    """Emit config copies for fej_on / fej_off (toggle use_fej). Returns {setting: path}.
    The exact_jacobian_sim setting is handled separately via the simulator."""
    raise NotImplementedError("TODO(intern): copy base config, set use_fej true/false")


def attributable_to_covariance(nees_fej_on: float, nees_fej_off: float, nees_exact_sim: float,
                               n: int, threshold_ratio: float = 5.0) -> bool:
    """If nees/n stays >> 1 even with FEJ on AND in the exact-Jacobian sim, the explosion is
    covariance-driven, not linearisation-driven (Section 15)."""
    persists = (nees_fej_on / n > threshold_ratio) and (nees_exact_sim / n > threshold_ratio)
    print(f"  nees/n: fej_on={nees_fej_on/n:.1f}, fej_off={nees_fej_off/n:.1f}, "
          f"exact_sim={nees_exact_sim/n:.1f}; persists under FEJ+exact: {persists}")
    return persists


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
