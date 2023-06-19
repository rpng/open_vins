#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# datasets
datasets=(
#    "udel_gore"
#    "udel_arl"
#    "udel_gore_zupt"
    "tum_corridor1_512_16_okvis"
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
save_path1="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_calib/algorithms"
save_path2="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_calib/timings"
save_path3="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_calib/truths"



#=============================================================
# Start the Monte Carlo Simulations
#=============================================================

big_start_time="$(date -u +%s)"

# Loop through the datasets
for h in "${!datasets[@]}"; do
# Loop through if we want to calibrate
for m in "${!sim_do_calibration[@]}"; do
# Loop through if we want to perturb
for n in "${!sim_do_perturbation[@]}"; do
# Monte Carlo runs for this dataset
for j in {00..49}; do

# SKIP: no calib and perturbing
if [ "${sim_do_calibration[m]}" == "false" ] && [ "${sim_do_perturbation[n]}" == "true" ]
then
    continue
fi


folder="VIO"
if [ "${sim_do_calibration[m]}" == "true" ]
then
    folder="${folder}_CALIB"
fi
if [ "${sim_do_perturbation[n]}" == "true" ]
then
    folder="${folder}_PERTURBED"
fi
filename_est="$save_path1/${folder}/${datasets[h]}/estimate_$j.txt"
filename_gt="$save_path3/${datasets[h]}.txt"

# launch the simulation script
start_time="$(date -u +%s)"
roslaunch ov_msckf simulation.launch \
  verbosity:="WARNING" \
  seed:="$((10#$j + 1))" \
  dataset:="${datasets[h]}.txt" \
  max_cameras:="1" \
  fej:="true" \
  sim_do_calibration:="${sim_do_calibration[m]}" \
  sim_do_perturbation:="${sim_do_perturbation[n]}" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - ${folder} - run $j took $elapsed seconds";


done
done
done
done



# print out the time elapsed
big_end_time="$(date -u +%s)"
big_elapsed="$(($big_end_time-$big_start_time))"
echo "BASH: script took $big_elapsed seconds in total!!";


