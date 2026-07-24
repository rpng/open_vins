#!/usr/bin/env python3
"""
ablation_a7_split_leakage.py  --  A7: frame-level vs sequence-disjoint calibration split.

Companion to: Section 14 (ablation table) and Section 9.1 (non-exchangeability).

WHAT IT ESTABLISHES (Section 14): quantifies how much coverage LEAKAGE an improper split
produces. This is the experimental face of the non-exchangeability contribution (Section 9.1):
trajectory data violate exchangeability because consecutive frames are near-duplicates, so a
calibration set built by randomly sampling FRAMES contains near-duplicates of test frames ->
the conformal guarantee appears to hold when it does not (FALSE coverage).

DESIGN: compute the conformal q_alpha two ways and compare realised coverage on a truly held-out
test:
    * frame-level split:        randomly sample frames into calib/test (LEAKY -- the wrong way)
    * sequence-disjoint split:  calib and test never share a trajectory (the correct way;
                                EUROC_SEQUENCE_DISJOINT_SPLIT in hdf5_dump_dataset.py)

MEASURED: nominal vs realised coverage under each split; the gap = the leakage.

SUCCESS (i.e. the point is demonstrated): the frame-level split reports coverage AT nominal on
its own (leaky) test but UNDER-covers on genuinely unseen sequences, while the sequence-disjoint
split's reported coverage matches reality. The size of the frame-level over-optimism is the
number this ablation contributes.

WHY IT MATTERS: it is the evidence for the Section 9.1 claim that the sequence-disjoint split
(which makes number-of-sequences the binding resource constraint) is NECESSARY, not fussy.

TODO(intern): reuse split_conformal_per_modality.fit_per_modality with two different split
constructions over the SAME dumps; measure coverage with metrics_nees_ate_rte.coverage.
"""

from __future__ import annotations

SPLIT_MODES = ("frame_level_leaky", "sequence_disjoint")


def run_a7(h5_dir: str, out_csv: str) -> None:
    raise NotImplementedError(
        "TODO(intern): fit q_alpha under both split modes; measure realised coverage on unseen "
        "sequences; report the leakage gap.")


def check_success(results) -> bool:
    """frame-level split over-reports coverage (leakage > threshold); sequence-disjoint is honest."""
    raise NotImplementedError("TODO(intern): encode the leakage gap comparison")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
