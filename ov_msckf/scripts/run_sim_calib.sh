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

# If we want to calibrate parameters
sim_do_calibration=(
    "false"
    "true"
)

# If we want to perturb the initial state values
sim_do_perturbation=(
    "false"
    "true"
)

# location to save log files into
save_path_est="/home/chuchu/test_ov/openvins_pra/sim_calib/algorithms"
save_path_gt="/home/chuchu/test_ov/openvins_pra/sim_calib/truths"

#=============================================================
# Start the Monte Carlo Simulations
#=============================================================

# Loop through the datasets
for h in "${!datasets[@]}"; do
# Loop through if we want to calibrate
for m in "${!sim_do_calibration[@]}"; do
# Loop through if we want to perturb
for n in "${!sim_do_perturbation[@]}"; do
# Monte Carlo runs for this dataset
for j in {00..02}; do


filename_est="$save_path_est/calib_${sim_do_calibration[m]}/perturb_${sim_do_perturbation[n]}/${datasets[h]}/estimate_$j.txt"
filename_gt="$save_path_gt/${datasets[h]}.txt"

#===============================================
# Start Monte Carlo Simulations
#===============================================
start_time="$(date -u +%s)"
roslaunch ov_msckf simulation.launch \
  seed:="$((10#$j + 1))" \
  dataset:="${datasets[h]}.txt" \
  sim_do_calibration:="${sim_do_calibration[m]}" \
  sim_do_perturbation:="${sim_do_perturbation[n]}" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - calib_${sim_do_calibration[m]} - perturb_${sim_do_perturbation[n]} - run $j took $elapsed seconds";
#===============================================
#===============================================

done
done
done
done

