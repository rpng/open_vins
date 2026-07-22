#!/usr/bin/env python3
"""
ablation_a4_scalar_vs_per_feature_R.py  --  A4: scalar vs per-feature R.

Companion to: Section 14 (ablation table) and Section 13.3. PROMOTED to the main paper under
the ICRA plan, where it carries C2's argument (C2 itself is deferred to the journal).

WHAT IT ESTABLISHES (Section 14): justifies Net A's set architecture. If one scalar sufficed,
the 26k-parameter DeepSets head would be unnecessary complexity.

DESIGN: on the CORRUPTION SUITE (not UZH-FPV -- that's C2), compare three R models:
    * stock scalar R            (single global sigma_pix, the OpenVINS default)
    * learned scalar R          (Net A outputs averaged to one value per frame)
    * learned per-feature R     (full Net A)
Degraded frames are precisely where feature quality becomes heterogeneous, so this is where a
per-feature R should earn its keep (Section 13.3, "ablation A4 ... substitutes").

MEASURED: nees/n and ATE per severity per arm; the WITHIN-FRAME spread of predicted sigma.

SUCCESS: per-feature beats both scalar variants on nees, with a visibly non-trivial within-frame
sigma spread (the network discriminates good vs bad features, not just a per-frame average).

TODO(intern): the 'learned scalar R' arm is Net A with per-feature outputs mean-pooled per frame
before injection; reuse plug_learned_sigma.py with an averaging flag.
"""

from __future__ import annotations

ARMS = ("stock_scalar_R", "learned_scalar_R", "learned_per_feature_R")


def run_a4(h5_dir: str, out_csv: str) -> None:
    raise NotImplementedError(
        "TODO(intern): corruption-suite sweep over ARMS; record nees/n, ATE, within-frame sigma spread.")


def check_success(results) -> bool:
    """per_feature nees < both scalar arms, and within-frame sigma spread > threshold."""
    raise NotImplementedError("TODO(intern): encode per-feature-wins-on-nees + non-trivial spread")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
