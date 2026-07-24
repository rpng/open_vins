#!/usr/bin/env python3
"""
make_money_plot.py  --  the one figure that carries the paper.

Companion to: Section 16 ("The money plot").

THE FIGURE (Section 16):
    x-axis: degradation severity (0..5 from the corruption suite).
    three curves of nees/n vs severity:
        naive learned  -- nees explodes.
        conformalised  -- graceful.
        stock OpenVINS -- flat but blunt (honest because it never claimed much).
    overlay: the Theorem-2 crossover threshold (2*kappa/(kappa-1)) as a VERTICAL line at the
             severity where the estimated overconfidence a1 first exceeds it.

"If that vertical line lands where the naive curve turns upward, the figure says: we derived
from first principles where this system would break, and it broke there." Theory forecasting
the failure point of a real filter is what separates a spotlight from a poster (Section 16 /
claim C3b, Section 13.4).

INPUT: a results table (one row per (arm, severity, seed)) with nees/n and ATE, plus the
per-severity estimated a1/kappa from C3b. Report medians with interquartile ranges across
seeds (single-run VIO numbers are noisy -- Section 13.1 protocol).

TODO(intern): load the aggregated results (produced by the claim drivers), draw the 3 curves
with IQR bands, the chi-squared consistency band around nees/n = 1 (metrics.chi2_consistency_bounds),
and the Theorem-2 vertical line. This stub sets up the axes and the intended layout.
"""

from __future__ import annotations

import argparse


def make_money_plot(results_csv: str, crossover_severity: float, out_path: str) -> None:
    """Render the money plot from an aggregated results CSV.

    Expected columns: arm, severity, seed, nees_over_n, ate. Arms: 'naive', 'conformal', 'stock'.
    crossover_severity: the severity at which estimated a1 crosses 2*kappa/(kappa-1) (from C3b).
    """
    import matplotlib.pyplot as plt  # local import: only needed to draw.
    import numpy as np
    import pandas as pd

    df = pd.read_csv(results_csv)
    fig, ax = plt.subplots(figsize=(6, 4))

    styles = {"naive": ("Naive learned", "C3"),
              "conformal": ("Conformalised", "C0"),
              "stock": ("Stock OpenVINS", "C7")}
    for arm, (label, color) in styles.items():
        sub = df[df.arm == arm]
        if sub.empty:
            continue  # TODO(intern): fill once the claim drivers produce results
        g = sub.groupby("severity").nees_over_n
        med, lo, hi = g.median(), g.quantile(0.25), g.quantile(0.75)
        ax.plot(med.index, med.values, "-o", color=color, label=label)
        ax.fill_between(med.index, lo.values, hi.values, color=color, alpha=0.2)

    ax.axhline(1.0, ls=":", color="k", lw=1, label="consistent (nees/n = 1)")
    ax.axvline(crossover_severity, ls="--", color="C1", lw=2,
               label="Theorem 2 crossover")  # the overlay that makes it a spotlight
    ax.set_xlabel("degradation severity")
    ax.set_ylabel("nees / n")
    ax.set_yscale("log")  # nees can explode by orders of magnitude
    ax.legend(fontsize=8)
    ax.set_title("conformal money plot (Section 16)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    print(f"wrote {out_path}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Render the conformal money plot")
    ap.add_argument("--results", required=True, help="aggregated results CSV from the claim drivers")
    ap.add_argument("--crossover-severity", type=float, required=True,
                    help="severity where estimated a1 crosses the Theorem-2 threshold (C3b)")
    ap.add_argument("--out", default="money_plot.png")
    args = ap.parse_args()
    make_money_plot(args.results, args.crossover_severity, args.out)


if __name__ == "__main__":
    main()
