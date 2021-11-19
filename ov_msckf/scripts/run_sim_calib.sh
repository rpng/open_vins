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
#    "udel_neighborhood"
)

# location to save log files into
save_path_est="/home/cc/test/openvins_pra/sim_calib/algorithms"
save_path_gt="/home/cc/test/openvins_pra/sim_calib/truths"

#=============================================================
# Start the Monte Carlo Simulations
#=============================================================

# Loop through if use fej or not
for h in "${!datasets[@]}"; do

# groundtruth file save location
filename_gt="$save_path_gt/${datasets[h]}.txt"

# Monte Carlo runs for this dataset
for j in {00..02}; do

#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_static_groundtruth/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf simulation.launch \
  seed:="$j" dataset:="${datasets[h]}.txt" \
  sim_do_calibration:="false" \
  sim_do_perturbation:="false" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - static_groundtruth - run $j took $elapsed seconds";

#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_calib_groundtruth/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf simulation.launch \
  seed:="$j" \
  dataset:="${datasets[h]}.txt" \
  sim_do_calibration:="true" \
  sim_do_perturbation:="false" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - calib_groundtruth - run $j took $elapsed seconds";

#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_calib_perturbed/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf simulation.launch \
  seed:="$j" \
  dataset:="${datasets[h]}.txt" \
  sim_do_calibration:="true" \
  sim_do_perturbation:="true" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - calib_perturbed - run $j took $elapsed seconds";

#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_static_perturbed/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf simulation.launch \
  seed:="$j" \
  dataset:="${datasets[h]}.txt" \
  sim_do_calibration:="false" \
  sim_do_perturbation:="true" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - static_perturbed - run $j took $elapsed seconds";
#===============================================
#===============================================


done
done

