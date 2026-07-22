#!/usr/bin/env python3
"""
plug_learned_sigma.py  --  Stage 3: put the learned (and conformalised) sigma back into Q, R.

Companion to: Section 17 (the decoupled 3-stage pipeline) step "Stage 3: learned sigma -> back
              into OpenVINS Q, R -> evaluate", and Section 8.3 step 4 ("Freeze. Plug the
              predicted sigma back in as Q and R").

THE MECHANISM (and why it keeps OpenVINS frozen)
------------------------------------------------
conformal changes only five numbers (Section 4.4): the 4 IMU noise densities inside Q (Net B)
and the per-feature pixel scalar inside R (Net A). Because the filter is frozen and no gradient
flows through it (Section 17), the cleanest reproducible way to inject the learned sigma is a
SIDECAR file, computed fully offline:

    1. Load frozen Net A + Net B (Stage 2 checkpoints).
    2. Run them over the Stage-1 diagnostics for the TEST sequences (optionally corrupted).
    3. Apply the per-modality conformal scaler (split_conformal_per_modality.py).
    4. Write a "sigma injection" HDF5 keyed by (frame_time -> 4 IMU sigmas) and
       (frame_time, feature_id -> sigma_pix). This is what a Stage-3 OpenVINS run reads
       instead of the constants in NoiseManager / UpdaterOptions.

The Stage-3 C++ side is a thin variant of run_asl_msckf.cpp that, per frame, looks up these
sigmas and sets them before propagate/update -- still not touching the filter's algorithms,
only the two constants the paper says it is allowed to touch. See changes.md, "Stage 3 wiring".

ARMS this produces (used by the claim/ablation drivers):
    stock          : constants untouched (baseline A1)
    learned         : nets, no conformal (arm ii of C0; ablation A2)
    conformalised   : nets + conformal scaler (the repair; C1/C3)
    oracle          : sigma = |true error| from GT (achievable floor; arm iii of C0)

TODO(intern): implement net loading (TorchScript export recommended so the C++ runner can also
call them live if you prefer that to the sidecar), the forward passes over dumped diagnostics,
and the HDF5 writer. Keep the sidecar schema identical across arms so the C++ runner is arm-agnostic.
"""

from __future__ import annotations

import argparse
from enum import Enum


class Arm(str, Enum):
    STOCK = "stock"
    LEARNED = "learned"
    CONFORMALISED = "conformalised"
    ORACLE = "oracle"


def build_sigma_sidecar(h5_dump: str, net_a_ckpt: str, net_b_ckpt: str,
                        conformal_json: str | None, arm: Arm, out_h5: str) -> None:
    """Produce the per-(frame,feature) sigma injection file for a given arm.

    Args:
        h5_dump:      Stage-1 diagnostics HDF5 for the (possibly corrupted) test sequence.
        net_a_ckpt:   frozen Net A weights (ignored for STOCK/ORACLE).
        net_b_ckpt:   frozen Net B weights (ignored for STOCK/ORACLE).
        conformal_json: fitted per-modality q_alpha (only for CONFORMALISED).
        arm:          which arm to synthesise.
        out_h5:       output sidecar path.
    """
    raise NotImplementedError(
        "TODO(intern): 1) load nets, 2) forward over dumped diagnostics, 3) apply conformal "
        "scaler if arm==CONFORMALISED, 4) for STOCK write the config constants, for ORACLE "
        "write |GT error|, 5) dump sidecar HDF5.")


def main() -> None:
    ap = argparse.ArgumentParser(description="conformal Stage-3 sigma injector")
    ap.add_argument("--h5-dump", required=True)
    ap.add_argument("--net-a", default="")
    ap.add_argument("--net-b", default="")
    ap.add_argument("--conformal", default="")
    ap.add_argument("--arm", type=Arm, choices=list(Arm), required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    build_sigma_sidecar(args.h5_dump, args.net_a, args.net_b,
                        args.conformal or None, args.arm, args.out)


if __name__ == "__main__":
    main()
