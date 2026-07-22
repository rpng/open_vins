#!/usr/bin/env python3
"""
ablation_a6_netb_std_pool.py  --  A6: Net B without standard-deviation pooling.

Companion to: Section 14 (ablation table) and Section 8.2 (the std-pool motivation).

WHAT IT ESTABLISHES (Section 14): tests the PHYSICAL claim that the within-window standard
deviation is the fingerprint of un-integrable IMU data. Vibration and clipping live in the
HIGH-FREQUENCY content of the IMU signal; mean and max destroy that -- a propeller resonance
and a smooth manoeuvre can share a mean (Section 8.2). The std across the window is what
distinguishes them.

DESIGN: train two Net B variants and compare, specifically under the INERTIAL corruptions that
are high-frequency by construction (vibration; also clipping) -- see corruptions_inertial.py.
    * with std-pool:    NetB(use_std_pool=True)   [mean; max; std] pool  (the default)
    * without std-pool: NetB(use_std_pool=False)  [mean; max] pool only
NetB already exposes the `use_std_pool` flag exactly for this ablation
(conformal/stage2_train/net_b_imu_dilated_tcn.py).

MEASURED: Net B's predicted-vs-true noise-density calibration, and downstream nees/n, under the
vibration corruption sweep (the failure the std-pool is designed to detect).

SUCCESS: the std-pool variant is materially better at detecting vibration/clipping severity
(lower NLL, better nees under inertial corruption); removing it degrades exactly on the
high-frequency corruptions and roughly ties elsewhere -- isolating the std term's contribution.

TODO(intern): train both variants (train_heads.py with a flag), evaluate on the vibration and
clipping sweeps, compare.
"""

from __future__ import annotations

VARIANTS = ("with_std_pool", "without_std_pool")
FOCUS_CORRUPTIONS = ("vibration", "clipping")  # high-frequency; where std should matter most


def run_a6(h5_dir: str, out_csv: str) -> None:
    raise NotImplementedError(
        "TODO(intern): train NetB(use_std_pool=True/False); evaluate under FOCUS_CORRUPTIONS; compare.")


def check_success(results) -> bool:
    """std-pool wins on the high-frequency (vibration/clipping) sweeps."""
    raise NotImplementedError("TODO(intern): encode std-pool advantage under high-freq corruption")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
