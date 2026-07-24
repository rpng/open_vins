#!/usr/bin/env python3
"""Plot the local and published EuRoC ATE comparison."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter


ROOT = Path(__file__).resolve().parent
CSV_PATH = ROOT / "euroc_ate_comparison.csv"
PNG_PATH = ROOT / "euroc_ate_comparison.png"
PDF_PATH = ROOT / "euroc_ate_comparison.pdf"

METHODS = {
    "orb_slam3_stereo_inertial": {
        "label": "ORB-SLAM3 SI [published]",
        "short": "ORB-SLAM3",
        "color": "#2563EB",
        "linestyle": "--",
        "marker": "o",
    },
    "okvis2_causal_slam": {
        "label": "OKVIS2 causal SLAM [published]",
        "short": "OKVIS2 causal",
        "color": "#0284C7",
        "linestyle": "--",
        "marker": "s",
    },
    "okvis2_vio": {
        "label": "OKVIS2 VIO [published]",
        "short": "OKVIS2 VIO",
        "color": "#0D9488",
        "linestyle": "--",
        "marker": "^",
    },
    "openvins_full_sequence": {
        "label": "OpenVINS stock, full sequence [local]",
        "short": "OpenVINS full stock",
        "color": "#374151",
        "linestyle": "-",
        "marker": "D",
    },
    "stage3_stock": {
        "label": "OpenVINS Stage-3 stock [local]",
        "short": "Stage-3 stock",
        "color": "#16A34A",
        "linestyle": "-",
        "marker": "P",
    },
    "vins_fusion_stereo_inertial": {
        "label": "VINS-Fusion SI [published]",
        "short": "VINS-Fusion",
        "color": "#7C3AED",
        "linestyle": "--",
        "marker": "v",
    },
    "stage3_learned": {
        "label": "OpenVINS Stage-3 learned [local]",
        "short": "Stage-3 learned",
        "color": "#F59E0B",
        "linestyle": "-",
        "marker": "X",
    },
    "stage3_conformalised": {
        "label": "OpenVINS Stage-3 conformalised [local]",
        "short": "Stage-3 conformal",
        "color": "#DC2626",
        "linestyle": "-",
        "marker": "*",
    },
}


def load_rows() -> tuple[list[dict[str, str]], dict[str, str]]:
    with CSV_PATH.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or rows[-1]["sequence"] != "mean":
        raise ValueError(f"{CSV_PATH} must end with a mean row")
    return rows[:-1], rows[-1]


def main() -> None:
    rows, mean_row = load_rows()
    sequences = [
        "MH01", "MH02", "MH03", "MH04", "MH05",
        "V101", "V102", "V103", "V201", "V202", "V203",
    ]

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.titleweight": "bold",
        "axes.titlesize": 13,
        "axes.labelsize": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.facecolor": "white",
        "axes.facecolor": "#FAFAFA",
    })

    fig = plt.figure(figsize=(17, 8.8))
    fig.subplots_adjust(
        left=0.075, right=0.985, top=0.84, bottom=0.265, wspace=0.29,
    )
    grid = fig.add_gridspec(1, 2, width_ratios=(0.88, 1.55))
    ax_mean = fig.add_subplot(grid[0, 0])
    ax_sequence = fig.add_subplot(grid[0, 1])

    ranked = sorted(METHODS, key=lambda key: float(mean_row[key]))
    values = [float(mean_row[key]) for key in ranked]
    labels = [METHODS[key]["short"] for key in ranked]
    colors = [METHODS[key]["color"] for key in ranked]
    bars = ax_mean.barh(labels, values, color=colors, height=0.68)
    ax_mean.invert_yaxis()
    ax_mean.set_xlim(0, max(values) * 1.23)
    ax_mean.set_xlabel("Mean ATE RMSE over 11 sequences (m)")
    ax_mean.set_title("A. Overall accuracy ranking\n(lower is better)", loc="left")
    ax_mean.grid(axis="x", color="#D1D5DB", linewidth=0.7, alpha=0.8)
    ax_mean.set_axisbelow(True)
    for bar, value in zip(bars, values):
        ax_mean.text(
            value + max(values) * 0.018,
            bar.get_y() + bar.get_height() / 2,
            f"{value:.3f}",
            va="center",
            ha="left",
            fontsize=9.5,
            fontweight="bold",
            color="#111827",
        )

    held_out = {4, 9, 10}
    for index in held_out:
        ax_sequence.axvspan(
            index - 0.45, index + 0.45,
            color="#FEF3C7", alpha=0.55, linewidth=0, zorder=0,
        )

    x_values = list(range(len(sequences)))
    for key, style in METHODS.items():
        y_values = [float(row[key]) for row in rows]
        is_local = key.startswith("stage3_") or key == "openvins_full_sequence"
        ax_sequence.plot(
            x_values,
            y_values,
            label=style["label"],
            color=style["color"],
            linestyle=style["linestyle"],
            linewidth=2.25 if is_local else 1.75,
            marker=style["marker"],
            markersize=6.5,
            markeredgecolor="white",
            markeredgewidth=0.6,
            alpha=0.98,
            zorder=3 if is_local else 2,
        )

    ax_sequence.set_yscale("log")
    ax_sequence.set_ylim(0.01, 1.05)
    ax_sequence.set_yticks([0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0])
    ax_sequence.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    ax_sequence.set_xticks(x_values, sequences, rotation=35, ha="right")
    ax_sequence.set_ylabel("ATE RMSE (m, logarithmic scale)")
    ax_sequence.set_title(
        "B. Per-sequence accuracy\n"
        "(shaded columns are the held-out conformal test split)",
        loc="left",
    )
    ax_sequence.grid(which="major", axis="y", color="#D1D5DB", linewidth=0.75)
    ax_sequence.grid(which="minor", axis="y", color="#E5E7EB", linewidth=0.45, alpha=0.7)
    ax_sequence.set_axisbelow(True)

    handles, legend_labels = ax_sequence.get_legend_handles_labels()
    fig.legend(
        handles,
        legend_labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.088),
        ncol=4,
        frameon=False,
        fontsize=9.2,
        handlelength=2.8,
        columnspacing=1.5,
    )
    fig.suptitle(
        "EuRoC MAV: conformal OpenVINS versus popular stereo-inertial systems",
        fontsize=17,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.042,
        "Local OpenVINS runs use rigid SE(3) alignment. Published baselines come "
        "from ORB-SLAM3 Table II and OKVIS2 Table I and are not same-hardware reruns.",
        ha="center",
        va="center",
        fontsize=9,
        color="#4B5563",
    )

    fig.savefig(PNG_PATH, dpi=220, bbox_inches="tight")
    fig.savefig(PDF_PATH, bbox_inches="tight")
    print(PNG_PATH)
    print(PDF_PATH)


if __name__ == "__main__":
    main()
