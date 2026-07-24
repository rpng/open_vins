#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${OPENVINS_IMAGE:-openvins-euroc:noetic}"
DATA_ROOT="${DATA_ROOT:-$REPO_DIR/EuRoC_MAV}"
RESULTS_ROOT="${RESULTS_ROOT:-$REPO_DIR/openvins_benchmark}"
T_MAX_DIFF="${T_MAX_DIFF:-0.02}"
RPE_DELTA="${RPE_DELTA:-1}"

ACTION="all"
ONLY_SEQUENCE=""
RERUN=0

SEQUENCES=(
  "MH_01_easy|machine_hall|40"
  "MH_02_easy|machine_hall|35"
  "MH_03_medium|machine_hall|5"
  "MH_04_difficult|machine_hall|10"
  "MH_05_difficult|machine_hall|5"
  "V1_01_easy|vicon_room1|0"
  "V1_02_medium|vicon_room1|0"
  "V1_03_difficult|vicon_room1|0"
  "V2_01_easy|vicon_room2|0"
  "V2_02_medium|vicon_room2|0"
  "V2_03_difficult|vicon_room2|0"
)

usage() {
  cat <<USAGE
Usage: $0 [action] [options]

Actions:
  all          Build image, run trajectories, and benchmark (default)
  build        Build the OpenVINS Docker image
  run          Run OpenVINS only
  benchmark    Benchmark existing trajectories only

Options:
  --sequence NAME  Process one EuRoC sequence
  --rerun          Replace an existing estimate by running it again
  -h, --help       Show this help
USAGE
}

if [[ $# -gt 0 && "$1" != -* ]]; then
  ACTION="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence)
      ONLY_SEQUENCE="${2:-}"
      [[ -n "$ONLY_SEQUENCE" ]] || { echo "--sequence requires a value" >&2; exit 2; }
      shift 2
      ;;
    --rerun)
      RERUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

case "$ACTION" in
  all|build|run|benchmark) ;;
  *) echo "unknown action: $ACTION" >&2; usage; exit 2 ;;
esac

log() {
  printf '\n[%s] %s\n' "$(date '+%F %T')" "$*"
}

selected_entries() {
  local entry seq group start
  for entry in "${SEQUENCES[@]}"; do
    IFS='|' read -r seq group start <<< "$entry"
    if [[ -n "$ONLY_SEQUENCE" && "$seq" != "$ONLY_SEQUENCE" ]]; then
      continue
    fi
    printf '%s|%s|%s\n' "$seq" "$group" "$start"
  done
}

ensure_selected() {
  if [[ -n "$ONLY_SEQUENCE" ]] && ! selected_entries | grep -q .; then
    echo "unknown EuRoC sequence: $ONLY_SEQUENCE" >&2
    exit 2
  fi
}

build_image() {
  log "Building $IMAGE"
  docker build \
    --file "$REPO_DIR/benchmark/Dockerfile.noetic" \
    --tag "$IMAGE" \
    "$REPO_DIR"
}

ensure_image() {
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    build_image
  fi
}

bag_path() {
  local seq="$1" group="$2"
  printf '%s/%s/%s/%s.bag' "$DATA_ROOT" "$group" "$seq" "$seq"
}

resolve_data_group() {
  local group="$1"
  if [[ -d "$DATA_ROOT/$group" ]]; then
    printf '%s\n' "$group"
    return
  fi
  case "$group" in
    vicon_room1) group="vicon_1" ;;
    vicon_room2) group="vicon_2" ;;
  esac
  printf '%s\n' "$group"
}

run_sequence() {
  local seq="$1" group="$2" start="$3"
  local data_group bag out estimate started ended
  data_group="$(resolve_data_group "$group")"
  bag="$(bag_path "$seq" "$data_group")"
  out="$RESULTS_ROOT/results/$seq"
  estimate="$out/estimate.txt"

  if [[ ! -s "$bag" ]]; then
    echo "WARNING: skipping $seq; bag is missing: $bag" >&2
    return 0
  fi
  if [[ "$RERUN" -eq 0 && -s "$estimate" ]]; then
    log "Skipping $seq; estimate already exists"
    return 0
  fi

  mkdir -p "$out/ros_logs"
  started="$(date +%s)"
  log "Running OpenVINS stereo-inertial on $seq (bag start ${start}s)"
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    --env HOME=/tmp \
    --env ROS_HOSTNAME=localhost \
    --env ROS_MASTER_URI=http://localhost:11311 \
    --env "ROS_LOG_DIR=/results/$seq/ros_logs" \
    --mount "type=bind,src=$DATA_ROOT,dst=/datasets,readonly" \
    --mount "type=bind,src=$RESULTS_ROOT/results,dst=/results" \
    --entrypoint /catkin_ws/devel/env.sh \
    "$IMAGE" \
    roslaunch ov_msckf serial.launch \
      config:=euroc_mav \
      dataset:="$seq" \
      bag:="/datasets/$data_group/$seq/$seq.bag" \
      bag_start:="$start" \
      max_cameras:=2 \
      use_stereo:=true \
      dosave:=true \
      path_est:="/results/$seq/estimate.txt" \
      dotime:=true \
      path_time:="/results/$seq/timing.txt" \
      dolivetraj:=false \
      >"$out/openvins.log" 2>&1
  ended="$(date +%s)"

  if [[ ! -s "$estimate" ]]; then
    echo "ERROR: OpenVINS did not generate $estimate" >&2
    return 1
  fi
  printf '%s\n' "$((ended - started))" > "$out/runtime_seconds.txt"
  log "$seq produced $(grep -vc '^#' "$estimate") poses in $((ended - started)) seconds"
}

metric() {
  local name="$1" file="$2"
  awk -v key="$name" '$1 == key { print $2; exit }' "$file"
}

benchmark_sequence() {
  local seq="$1" group="$2" summary_file="$3"
  local data_group out estimate gt_csv gt_tum est_tum ape_txt rpe_txt
  local rows runtime ape_rmse ape_mean ape_median ape_std rpe_rmse
  out="$RESULTS_ROOT/results/$seq"
  estimate="$out/estimate.txt"
  gt_csv="$REPO_DIR/ov_data/euroc_mav/$seq.csv"
  gt_tum="$out/gt.tum"
  est_tum="$out/estimate.tum"
  ape_txt="$out/ape.txt"
  rpe_txt="$out/rpe.txt"
  data_group="$(resolve_data_group "$group")"

  if [[ ! -s "$estimate" ]]; then
    echo "WARNING: skipping benchmark for $seq; estimate is missing" >&2
    return 0
  fi
  if [[ ! -s "$gt_csv" ]]; then
    echo "WARNING: skipping benchmark for $seq; ground truth is missing" >&2
    return 0
  fi

  log "Benchmarking $seq"
  # evo prompts before replacing plots/archives, which breaks unattended reruns.
  # These are derived artifacts and are regenerated immediately below.
  rm -f "$ape_txt" "$rpe_txt" \
    "$out/ape.zip" "$out/rpe.zip" \
    "$out/ape.pdf" "$out/rpe.pdf"
  python3 "$REPO_DIR/benchmark/trajectory_to_tum.py" euroc "$gt_csv" "$gt_tum" >/dev/null
  python3 "$REPO_DIR/benchmark/trajectory_to_tum.py" openvins "$estimate" "$est_tum" >/dev/null

  docker run --rm \
    --network none \
    --user "$(id -u):$(id -g)" \
    --env HOME=/tmp \
    --env MPLBACKEND=Agg \
    --mount "type=bind,src=$RESULTS_ROOT/results,dst=/results" \
    --entrypoint /catkin_ws/devel/env.sh \
    "$IMAGE" \
    evo_ape tum "/results/$seq/gt.tum" "/results/$seq/estimate.tum" \
      -a --t_max_diff "$T_MAX_DIFF" \
      --save_results "/results/$seq/ape.zip" \
      --save_plot "/results/$seq/ape.pdf" \
      >"$ape_txt"

  docker run --rm \
    --network none \
    --user "$(id -u):$(id -g)" \
    --env HOME=/tmp \
    --env MPLBACKEND=Agg \
    --mount "type=bind,src=$RESULTS_ROOT/results,dst=/results" \
    --entrypoint /catkin_ws/devel/env.sh \
    "$IMAGE" \
    evo_rpe tum "/results/$seq/gt.tum" "/results/$seq/estimate.tum" \
      -a --t_max_diff "$T_MAX_DIFF" \
      --delta "$RPE_DELTA" --delta_unit m \
      --save_results "/results/$seq/rpe.zip" \
      --save_plot "/results/$seq/rpe.pdf" \
      >"$rpe_txt"

  rows="$(wc -l < "$est_tum")"
  runtime="$(cat "$out/runtime_seconds.txt" 2>/dev/null || echo nan)"
  ape_rmse="$(metric rmse "$ape_txt")"
  ape_mean="$(metric mean "$ape_txt")"
  ape_median="$(metric median "$ape_txt")"
  ape_std="$(metric std "$ape_txt")"
  rpe_rmse="$(metric rmse "$rpe_txt")"
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$seq" "$data_group" "$rows" "$runtime" "$ape_rmse" "$ape_mean" \
    "$ape_median" "$ape_std" "$rpe_rmse" >> "$summary_file"
}

run_selected() {
  local seq group start
  while IFS='|' read -r seq group start; do
    run_sequence "$seq" "$group" "$start"
  done < <(selected_entries)
}

benchmark_selected() {
  local seq group start summary_file
  summary_file="$RESULTS_ROOT/summary.csv"
  if [[ -n "$ONLY_SEQUENCE" ]]; then
    summary_file="$RESULTS_ROOT/summary_${ONLY_SEQUENCE}.csv"
  fi
  printf '%s\n' \
    'sequence,group,poses,runtime_s,ape_rmse_m,ape_mean_m,ape_median_m,ape_std_m,rpe_rmse_m' \
    > "$summary_file"
  while IFS='|' read -r seq group start; do
    benchmark_sequence "$seq" "$group" "$summary_file"
  done < <(selected_entries)
  log "Summary"
  column -s, -t "$summary_file" 2>/dev/null || cat "$summary_file"
}

main() {
  ensure_selected
  mkdir -p "$RESULTS_ROOT/results"
  case "$ACTION" in
    build)
      build_image
      ;;
    run)
      ensure_image
      run_selected
      ;;
    benchmark)
      ensure_image
      benchmark_selected
      ;;
    all)
      ensure_image
      run_selected
      benchmark_selected
      ;;
  esac
}

main
