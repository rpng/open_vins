#!/usr/bin/env bash
#
# ROS2 serial batch runner for EuRoC MAV datasets.
# Reads bag path and output paths from environment (with fallbacks):
#   OV_WORKSPACE_ROOT- colcon workspace root (used when no first argument is passed)
#                   Only used to produce default paths if the three environment 
#                   variables below (OV_*) are not set.
#   OV_SAVE_PATH  - where to write estimate files (default: ./results)
#   OV_BAG_PATH   - directory containing dataset bags (default: ./datasets)
#   OV_VER        - version tag for output subdirs (default: 2.7)
#
# Usage: run_ros2_eth.sh [WS_ROOT]
#   WS_ROOT - optional; colcon workspace root.
#   If not passed: use OV_WORKSPACE_ROOT if set, else four levels up from this script.
#

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
# workspace root: first argument, else OV_WORKSPACE_ROOT env, else four levels up (workspace/.../open_vins/ov_msckf/scripts)
if [ -n "${1:-}" ]; then
  WS_ROOT="$(cd "$1" && pwd)"
elif [ -n "${OV_WORKSPACE_ROOT:-}" ]; then
  WS_ROOT="$(cd "${OV_WORKSPACE_ROOT}" && pwd)"
else
  WS_ROOT="$(cd "${SCRIPT_DIR}/../../../../" && pwd)"
fi

# source colcon workspace so ros2/ov_msckf is available in this script
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "BASH: install/setup.bash not found in ${WS_ROOT}. Set OV_WORKSPACE_ROOT or pass workspace root as first argument." >&2
  exit 1
fi

#-----------------------------------------------------------------------------
# Paths from ENV (fallbacks for local use)
#-----------------------------------------------------------------------------
save_path1="${OV_SAVE_PATH:-${WS_ROOT}/results}/euroc/algorithms"
save_path2="${OV_SAVE_PATH:-${WS_ROOT}/results}/euroc/timings"
bag_path="${OV_BAG_PATH:-${WS_ROOT}/datasets}/euroc"
ov_ver="${OV_VER:-2.7}"

# print paths
echo "BASH: estimates path = ${save_path1}"
echo "BASH: timings path = ${save_path2}"
echo "BASH: bags path = ${bag_path}"
echo "BASH: OpenVINS version = ${ov_ver}"

#-----------------------------------------------------------------------------
# Estimator modes and datasets
#-----------------------------------------------------------------------------
modes=(
  "mono"
  "binocular"
  "stereo"
)

bagnames=(
  "V1_01_easy"
  "V1_02_medium"
  "V1_03_difficult"
  "V2_01_easy"
  "V2_02_medium"
  "V2_03_difficult"
  "MH_01_easy"
  "MH_02_easy"
  "MH_03_medium"
  "MH_04_difficult"
  "MH_05_difficult"
)

bagstarttimes=(
  "0.0" "0.0" "0.0" "0.0" "0.0" "0.0"
  "40.0" "35.0" "5.0" "10.0" "5.0"
)

#-----------------------------------------------------------------------------
# Run
#-----------------------------------------------------------------------------
big_start_time="$(date -u +%s)"

for i in "${!bagnames[@]}"; do
  for h in "${!modes[@]}"; do
    for j in {00..00}; do
      start_time="$(date -u +%s)"
      filename_est="${save_path1}/ov_${ov_ver}_${modes[h]}/${bagnames[i]}/${j}_estimate.txt"
      filename_time="${save_path2}/ov_${ov_ver}_${modes[h]}/${bagnames[i]}/${j}_timing.txt"

      case "${modes[h]}" in
        mono)      temp1="1"; temp2="true"  ;;
        binocular) temp1="2"; temp2="false" ;;
        stereo)    temp1="2"; temp2="true"  ;;
        *)         echo "BASH: unknown mode ${modes[h]}"; exit 1 ;;
      esac

      # rosbag2: bag path is typically a directory per dataset (e.g. V1_01_easy/)
      bag_uri="${bag_path}/${bagnames[i]}"
      metadata_file="${bag_uri}/metadata.yaml"

      # skip datasets that are not available in this local checkout
      if [ ! -f "$metadata_file" ]; then
        echo "BASH: skipping ${bagnames[i]} (${modes[h]}) - missing rosbag2 metadata at ${metadata_file}"
        continue
      fi

      mkdir -p "$(dirname "$filename_est")" "$(dirname "$filename_time")"

      # launch and capture output
      launch_err="$(mktemp)"
      if ! ros2 launch ov_msckf serial.launch.py \
        max_cameras:="$temp1" \
        use_stereo:="$temp2" \
        config:=euroc_mav \
        dataset:="${bagnames[i]}" \
        bag:="$bag_uri" \
        bag_start:="${bagstarttimes[i]}" \
        dosave:=true \
        path_est:="$filename_est" \
        dotime:=true \
        debug:=false \
        path_time:="$filename_time" \
        >"$launch_err" 2>&1; then
        echo "BASH: ros2 launch failed for ${modes[h]} - ${bagnames[i]} (bag: $bag_uri)" >&2
        echo "BASH: captured output (stdout+stderr):" >&2
        cat "$launch_err" >&2
        rm -f "$launch_err"
        exit 1
      fi
      rm -f "$launch_err"

      end_time="$(date -u +%s)"
      elapsed=$((end_time - start_time))
      echo "BASH: ${modes[h]} - ${bagnames[i]} - run $j took $elapsed seconds"
    done
  done
done

big_end_time="$(date -u +%s)"
big_elapsed=$((big_end_time - big_start_time))
echo "BASH: script took $big_elapsed seconds in total!!"
