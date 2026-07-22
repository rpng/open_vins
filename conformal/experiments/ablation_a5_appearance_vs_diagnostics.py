#!/usr/bin/env python3
"""
ablation_a5_appearance_vs_diagnostics.py  --  A5: appearance vs tracker-diagnostic features.

Companion to: Section 14.2 ("A5 in detail -- why transfer is achievable"). MAIN PAPER.
Scheduled Week 5 alongside TUM-VI (Section 21).

DESIGN (Section 14.2): train TWO versions of Net A's head.
    Version 1 (appearance):  learned CNN features from the image patch around each corner.
    Version 2 (diagnostics): tracker diagnostics only -- KLT forward-backward error, corner
                             response, track age, parallax (the default conformal head).
Evaluate BOTH in-distribution on EuRoC and zero-shot on TUM-VI (a different building/hardware).

PREDICTION (Section 14.2): appearance tracks error well IN-DISTRIBUTION -- perhaps BETTER than
diagnostics, because it can exploit scene-specific regularities. It then FAILS TO TRANSFER,
because those regularities are the room, not the physics.

WHY A5 EARNS MAIN-PAPER SPACE (Section 14.2): it converts a design decision into a FINDING.
Without it, "we used tracker diagnostics" is an arbitrary implementation choice. With it, the
paper states a general principle: uncertainty heads that read SYMPTOMS transfer; uncertainty
heads that read APPEARANCE memorise. That principle generalises beyond VIO (thesis Chapter 4).

MEASURED: per-feature NLL / nees for both heads, EuRoC (in-dist) vs TUM-VI (zero-shot).

SUCCESS: appearance <= diagnostics in-distribution BUT appearance >> diagnostics (worse) on
TUM-VI zero-shot -- i.e. the transfer gap flips the ranking.

TODO(intern): the appearance head needs image patches dumped around each corner in Stage 1
(extend DiagnosticsLogger.hpp to save small patches, or re-crop from frames offline). The
diagnostics head is the stock NetA (conformal/stage2_train/net_a_visual_deepsets.py).
"""

from __future__ import annotations

HEADS = ("appearance", "diagnostics")
DATASETS = ("euroc_in_distribution", "tumvi_zero_shot")


def run_a5(h5_dir_euroc: str, h5_dir_tumvi: str, out_csv: str) -> None:
    raise NotImplementedError(
        "TODO(intern): train both heads on EuRoC; eval both on EuRoC and TUM-VI; record NLL/nees.")


def check_success(results) -> bool:
    """diagnostics transfers (small in->out gap); appearance memorises (large in->out gap)."""
    raise NotImplementedError("TODO(intern): encode the ranking-flip between in-dist and zero-shot")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
