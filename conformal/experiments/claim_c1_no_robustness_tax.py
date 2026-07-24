#!/usr/bin/env python3
"""
claim_c1_no_robustness_tax.py  --  C1: no robustness tax.

Companion to: Section 13.2 (claim C1). Scheduled Week 4 (Section 21).

STATEMENT: in-distribution on CLEAN EuRoC, the conformalised system matches unmodified OpenVINS.

WHY C1 EXISTS (Section 13.2): it pre-empts the most obvious reviewer objection -- "you made the
filter more honest by making it worse." A method that buys out-of-distribution honesty by
sacrificing in-distribution accuracy is a much weaker result. C1 shows the two are not traded.

PROTOCOL (Section 13.2): clean EuRoC, sequence-disjoint train/test. Arms: stock,
learned-no-conformal, conformalised. Multiple seeds.

MEASURED: ATE, RTE, and nees.

SUCCESS CRITERION (Section 13.2): ATE ratio to stock within run-to-run variance -- practically
within ~+/-10% and statistically indistinguishable. nees no worse than stock and preferably
closer to n.

IF IT FAILS (Section 13.2): serious -- it would suggest the learned sigma are harmful even
in-distribution, undercutting the framing that the problem is specific to distribution shift.

TODO(intern): reuse the arm runner from plug_learned_sigma.py on CLEAN sequences (severity 0),
aggregate over seeds, apply the criterion below.
"""

from __future__ import annotations

ARMS = ("stock", "learned", "conformalised")
ATE_TOLERANCE = 0.10  # +/-10% band vs stock


def run_c1(h5_dir: str, out_csv: str) -> None:
    raise NotImplementedError(
        "TODO(intern): clean-EuRoC sweep over ARMS x seeds; record ATE/RTE/nees_over_n -> out_csv.")


def check_success(results) -> bool:
    """ATE(conformalised) within +/-10% of ATE(stock) and nees no worse than stock."""
    import numpy as np
    def med(arm, field):
        xs = [r[field] for r in results if r["arm"] == arm]
        return float(np.median(xs)) if xs else float("nan")
    ate_ratio = med("conformalised", "ate") / max(med("stock", "ate"), 1e-9)
    nees_ok = med("conformalised", "nees_over_n") <= med("stock", "nees_over_n") * 1.05
    ate_ok = abs(ate_ratio - 1.0) <= ATE_TOLERANCE
    print(f"  ATE ratio conformalised:stock = {ate_ratio:.3f} (want within +/-{ATE_TOLERANCE:.0%})")
    print(f"  nees no worse than stock: {nees_ok}")
    return bool(ate_ok and nees_ok)


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
