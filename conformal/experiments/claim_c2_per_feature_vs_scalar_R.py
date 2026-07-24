#!/usr/bin/env python3
"""
claim_c2_per_feature_vs_scalar_R.py  --  C2: per-feature R beats scalar R under real
                                          aggressive motion.

Companion to: Section 13.3 (claim C2).

STATUS: DEFERRED to the journal version under the ICRA timeline (Section 13.3 / publication
strategy Section 23). Ablation A4 on the corruption suite SUBSTITUTES for it in the conference
paper, since degraded frames are precisely where feature quality becomes heterogeneous. This
stub exists so the intern knows where C2 lives; do NOT spend Week 1-7 time here.

STATEMENT: on UZH-FPV, assigning each feature its own sigma_pix outperforms OpenVINS' single
global pixel-noise constant.

WHY C2 EXISTS (Section 13.3): it justifies Net A's existence -- if one scalar were sufficient,
26k parameters of set network would be unnecessary complexity.

PROTOCOL (Section 13.3): zero-shot on UZH-FPV (first-person-view racing drone: high speed,
violent rotation, real motion blur -- Section 11). Arms:
    * stock scalar R;
    * learned scalar R (per-frame, one value);
    * learned per-feature R.
MEASURED: ATE, nees, and the spread of predicted sigma WITHIN each frame.

SUCCESS CRITERION (Section 13.3): per-feature beats both scalar variants on nees, WITH a visible
within-frame spread of predicted sigma -- showing the network genuinely discriminates good from
bad features, not just tracking a per-frame average.

TODO(intern, JOURNAL): needs UZH-FPV configs (config/uzhfpv_indoor*, already in the repo) and a
'learned scalar R' arm = Net A with its per-feature outputs averaged per frame.
"""

from __future__ import annotations

DEFERRED = True  # journal version; A4 substitutes for the conference paper (Section 13.3)
ARMS = ("stock_scalar_R", "learned_scalar_R", "learned_per_feature_R")


def run_c2(uzhfpv_h5_dir: str, out_csv: str) -> None:
    raise NotImplementedError("DEFERRED to journal (Section 13.3). See ablation_a4 for the conference substitute.")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
