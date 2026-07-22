#!/usr/bin/env bash
#
# gate2_reproduce_euroc_ate.sh  --  Gate 2: reproduce published EuRoC ATE with the default config.
#
# Companion to: Section 19.2 (Gate 2). Week 1 (Section 21). KILL GATE (Section 22): "Gates 1-2
# not green by end of Week 1 -> ICRA is off; revert to the IROS timeline."
#
# WHY THIS GATE EXISTS (Section 19.2): run UNMODIFIED OpenVINS with its shipped configuration and
# confirm the published trajectory-error numbers are reproduced. This validates the ENTIRE Stage-1
# chain -- data reading, timing, calibration, coordinate conventions -- against an external
# reference. Nothing downstream is valid until this passes: without it, a later poor result is
# uninterpretable (could be the science, could be a data-loading bug from three weeks earlier).
# "This gate is what makes every subsequent negative result trustworthy."
#
# GOOD NEWS: the repo already contains the machinery to do exactly this.
#   * benchmark/euroc_benchmark.sh      runs all 11 EuRoC sequences + computes APE(=ATE)/RPE(=RTE)
#   * openvins_benchmark/summary.csv    the reproduced numbers already committed (commit 3c279c4)
#   * benchmark/trajectory_to_tum.py    trajectory -> TUM for evaluation
# So Gate 2 is mostly: run that harness (or trust the committed run), then diff against the
# published OpenVINS EuRoC ATE table.
#
# USAGE:
#   DATA_ROOT=/path/to/EuRoC_MAV ./gate2_reproduce_euroc_ate.sh
#
# TODO(intern): point DATA_ROOT at your EuRoC download and set the expected published ATEs below
# (from the OpenVINS docs / paper). The script fails if any sequence deviates beyond TOL_RATIO.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DATA_ROOT="${DATA_ROOT:-$REPO_DIR/EuRoC_MAV}"
RESULTS_ROOT="${RESULTS_ROOT:-$REPO_DIR/openvins_benchmark}"
TOL_RATIO="${TOL_RATIO:-1.25}"   # allow 25% slack vs published (VIO runs are stochastic)

# Published EuRoC ATE (metres) -- TODO(intern): fill from the OpenVINS reference table.
declare -A PUBLISHED_ATE=(
  ["MH_01_easy"]="TODO"
  ["MH_02_easy"]="TODO"
  ["MH_03_medium"]="TODO"
  ["MH_04_difficult"]="TODO"
  ["MH_05_difficult"]="TODO"
  ["V1_01_easy"]="TODO"
  ["V1_02_medium"]="TODO"
  ["V1_03_difficult"]="TODO"
  ["V2_01_easy"]="TODO"
  ["V2_02_medium"]="TODO"
  ["V2_03_difficult"]="TODO"
)

echo "[gate2] Reproducing EuRoC ATE with the DEFAULT config (ov_msckf unmodified)."
echo "[gate2] Delegating the run to the existing harness: benchmark/euroc_benchmark.sh"
echo "[gate2]   DATA_ROOT=$DATA_ROOT  RESULTS_ROOT=$RESULTS_ROOT"

# Option A: run the full harness now.
#   DATA_ROOT="$DATA_ROOT" RESULTS_ROOT="$RESULTS_ROOT" "$REPO_DIR/benchmark/euroc_benchmark.sh" all
# Option B: trust the already-committed run in openvins_benchmark/summary.csv (commit 3c279c4).

# TODO(intern): parse ape.txt per sequence under $RESULTS_ROOT/results/<seq>/ and compare to
# PUBLISHED_ATE, failing (exit 1) if any ratio exceeds TOL_RATIO. Pseudo-logic:
#
#   for seq in "${!PUBLISHED_ATE[@]}"; do
#     got=$(grep -m1 'rmse' "$RESULTS_ROOT/results/$seq/ape.txt" | awk '{print $2}')
#     ratio=$(python3 -c "print($got/${PUBLISHED_ATE[$seq]})")
#     ...  fail if ratio > TOL_RATIO ...
#   done
#
echo "[gate2] TODO(intern): implement the comparison loop above and exit non-zero on deviation."
