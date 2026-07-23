#!/usr/bin/env bash
# Gate 2: compare a stock OpenVINS EuRoC run with a pinned reference table.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DATA_ROOT="${DATA_ROOT:-$REPO_DIR/EuRoC_MAV}"
RESULTS_ROOT="${RESULTS_ROOT:-$REPO_DIR/openvins_benchmark}"
REFERENCE_CSV="${REFERENCE_CSV:-$SCRIPT_DIR/openvins_euroc_reference.csv}"
TOL_RATIO="${TOL_RATIO:-1.25}"
RUN_BENCHMARK="${RUN_BENCHMARK:-0}"

if [[ "$RUN_BENCHMARK" == "1" ]]; then
  DATA_ROOT="$DATA_ROOT" RESULTS_ROOT="$RESULTS_ROOT" \
    "$REPO_DIR/benchmark/euroc_benchmark.sh" all
fi

SUMMARY="$RESULTS_ROOT/summary.csv"
for file in "$SUMMARY" "$REFERENCE_CSV"; do
  if [[ ! -s "$file" ]]; then
    echo "[gate2] ERROR: required file is missing or empty: $file" >&2
    exit 2
  fi
done

python3 - "$SUMMARY" "$REFERENCE_CSV" "$TOL_RATIO" <<'PY'
import csv
import math
import sys
from pathlib import Path

summary_path, reference_path = map(Path, sys.argv[1:3])
tolerance = float(sys.argv[3])
if tolerance < 1.0:
    raise SystemExit("[gate2] TOL_RATIO must be >= 1.0")


def read_table(path: Path, metric: str) -> dict[str, float]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"sequence", metric}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise SystemExit(f"[gate2] {path} must contain columns {sorted(required)}")
        values = {}
        for row in reader:
            sequence = row["sequence"].strip()
            value = float(row[metric])
            if not sequence or not math.isfinite(value) or value <= 0:
                raise SystemExit(f"[gate2] invalid row in {path}: {row}")
            values[sequence] = value
        return values


observed = read_table(summary_path, "ape_rmse_m")
reference = read_table(reference_path, "ape_rmse_m")
missing = sorted(set(reference) - set(observed))
extra = sorted(set(observed) - set(reference))
failed = []

print("sequence,reference_m,observed_m,ratio,status")
for sequence in sorted(reference):
    if sequence not in observed:
        continue
    ratio = observed[sequence] / reference[sequence]
    status = "PASS" if ratio <= tolerance else "FAIL"
    if status == "FAIL":
        failed.append(sequence)
    print(f"{sequence},{reference[sequence]:.6f},{observed[sequence]:.6f},{ratio:.3f},{status}")

if missing:
    print(f"[gate2] missing sequences: {', '.join(missing)}", file=sys.stderr)
if extra:
    print(f"[gate2] unexpected sequences: {', '.join(extra)}", file=sys.stderr)
if missing or extra or failed:
    if failed:
        print(f"[gate2] sequences over tolerance: {', '.join(failed)}", file=sys.stderr)
    raise SystemExit(1)
print(f"[gate2] PASS: {len(reference)} sequences are within {tolerance:.3f}x of the reference")
PY
