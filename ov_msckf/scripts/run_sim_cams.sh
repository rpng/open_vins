#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# datasets
datasets=(
      "udel_gore"
#    "udel_arl"
#    "udel_gore_zupt"
#    "tum_corridor1_512_16_okvis"
)

# number of cameras
cameras=(
    "1"
    "2"
    "3"
    "4"
)

# location to save log files into
save_path_est="/home/cc/test/openvins_pra/sim_cam/algorithms"
save_path_gt="/home/cc/test/openvins_pra/sim_cam/truths"

#=============================================================
# Start Monte-Carlo Simulations
#=============================================================
# Loop through datasets
for h in "${!datasets[@]}"; do
# Loop through number of cameras we want to use
for i in "${!cameras[@]}"; do
# Monte Carlo runs for this dataset
for j in {00..02}; do

# start timing
start_time="$(date -u +%s)"

# our filename
filename_est="$save_path_est/ov_v23_cam${cameras[i]}/${datasets[h]}/estimate_$j.txt"
filename_gt="$save_path_gt/${datasets[h]}.txt"

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf simulation.launch \
  seed:="$((10#$j + 1))" \
  max_cameras:="${cameras[i]}" \
  dataset:="${datasets[h]}.txt" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - ${cameras[i]} - run $j took $elapsed seconds";


done
done
done

