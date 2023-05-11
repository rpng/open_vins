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

# what modes to use
use_cross_cam=(
  "false"
  "true"
)

# number of cameras
cameras=(
    "1"
    "2"
    "3"
    "4"
)

# location to save log files into
save_path1="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_cams/algorithms"
save_path2="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_cams/timings"
save_path3="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_cams/truths"

#=============================================================
# Start Monte-Carlo Simulations
#=============================================================

big_start_time="$(date -u +%s)"

# Loop through datasets
for h in "${!datasets[@]}"; do
# Loop through number of cameras we want to use
for n in "${!use_cross_cam[@]}"; do
for i in "${!cameras[@]}"; do
# Monte Carlo runs for this dataset
for j in {00..49}; do

# start timing
start_time="$(date -u +%s)"

# skip the case were we are trying to do cross sensor
if [ "${cameras[i]}" == "1" ] && [ "${use_cross_cam[n]}" == "true" ]
then
  continue
fi

# our filename
temp=""
if [ "${use_cross_cam[n]}" == "true" ]
then
    temp="_cross"
fi
folder="cam${cameras[i]}${temp}"
filename_est="$save_path1/${folder}/${datasets[h]}/estimate_$j.txt"
filename_gt="$save_path3/${datasets[h]}.txt"

# launch the simulation script
roslaunch ov_msckf simulation.launch \
  verbosity:="WARNING" \
  seed:="$((10#$j + 1))" \
  dataset:="${datasets[h]}.txt" \
  use_stereo:="${use_cross_cam[n]}" \
  max_cameras:="${cameras[i]}" \
  fej:="true" \
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


