#!/usr/bin/env bash
# Run and validate conformal Stage-1 dumps for the complete EuRoC suite.

set -euo pipefail

IMAGE="${IMAGE:-openvins-conformal:stage1}"
DATA_ROOT="${DATA_ROOT:-/mnt/euro_mav}"
RESULTS_ROOT="${RESULTS_ROOT:-/mnt/euro_mav/conformal_dumps}"
CONFIG="${CONFIG:-/catkin_ws/src/open_vins/config/euroc_mav/estimator_config.yaml}"
RUNNER="/catkin_ws/devel/lib/conformal_stage1/run_asl_msckf"
VALIDATOR="/catkin_ws/src/open_vins/conformal/stage1_dumps/validate_dump.py"

SEQUENCES=(
  "MH_01_easy|40"
  "MH_02_easy|35"
  "MH_03_medium|5"
  "MH_04_difficult|10"
  "MH_05_difficult|5"
  "V1_01_easy|0"
  "V1_02_medium|0"
  "V1_03_difficult|0"
  "V2_01_easy|0"
  "V2_02_medium|0"
  "V2_03_difficult|0"
)

if (( $# == 0 )); then
  requested=(all)
else
  requested=("$@")
fi

is_requested() {
  local sequence="$1" item
  for item in "${requested[@]}"; do
    if [[ "$item" == "all" || "$item" == "$sequence" ]]; then
      return 0
    fi
  done
  return 1
}

validate_dump() {
  local dump="$1" report="$2"
  docker run --rm \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$RESULTS_ROOT:/results:ro" \
    "$IMAGE" python3 "$VALIDATOR" "/results/$(basename "$dump")" >"$report" 2>&1
}

mkdir -p "$RESULTS_ROOT"

for entry in "${SEQUENCES[@]}"; do
  IFS='|' read -r sequence start_offset <<<"$entry"
  is_requested "$sequence" || continue

  sequence_dir="$DATA_ROOT/$sequence"
  dump="$RESULTS_ROOT/${sequence}_stage1.h5"
  run_log="$RESULTS_ROOT/${sequence}_stage1.run.log"
  validation="$RESULTS_ROOT/${sequence}_stage1.validation.txt"
  runtime="$RESULTS_ROOT/${sequence}_stage1.runtime_seconds.txt"

  if [[ ! -f "$sequence_dir/mav0/cam0/data.csv" ]]; then
    echo "[stage1-suite] ERROR $sequence: missing ASL sequence at $sequence_dir" >&2
    exit 2
  fi

  if [[ -e "$dump" ]]; then
    if validate_dump "$dump" "$validation"; then
      echo "[stage1-suite] SKIP $sequence: existing dump passes validation"
      continue
    fi
    echo "[stage1-suite] ERROR $sequence: existing dump is invalid; refusing to overwrite $dump" >&2
    tail -40 "$validation" >&2
    exit 3
  fi

  echo "[stage1-suite] RUN $sequence start_offset_s=$start_offset"
  started="$(date +%s)"
  if ! docker run --rm --entrypoint bash \
      --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -v "$sequence_dir:/datasets/$sequence:ro" \
      -v "$RESULTS_ROOT:/results" \
      "$IMAGE" -lc \
      ". /opt/ros/noetic/setup.bash; . /catkin_ws/devel/setup.bash; exec $RUNNER $CONFIG /datasets/$sequence $sequence /results/${sequence}_stage1.h5 $start_offset" \
      >"$run_log" 2>&1; then
    echo "[stage1-suite] ERROR $sequence: runner failed" >&2
    tail -80 "$run_log" >&2
    exit 4
  fi
  elapsed="$(( $(date +%s) - started ))"
  printf '%s\n' "$elapsed" >"$runtime"

  if ! validate_dump "$dump" "$validation"; then
    echo "[stage1-suite] ERROR $sequence: dump validation failed" >&2
    tail -80 "$validation" >&2
    exit 5
  fi
  echo "[stage1-suite] PASS $sequence runtime_s=$elapsed size_bytes=$(stat -c %s "$dump")"
done

echo "[stage1-suite] COMPLETE"
