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
     "udel_arl"
#    "udel_gore_zupt"
#    "tum_corridor1_512_16_okvis"
)

# estimator configurations
usefej=(
    "false"
    "true"
)

# feature representations
representations=(
    "GLOBAL_3D"
    "GLOBAL_FULL_INVERSE_DEPTH"
    "ANCHORED_3D"
    "ANCHORED_FULL_INVERSE_DEPTH"
    "ANCHORED_MSCKF_INVERSE_DEPTH"
    "ANCHORED_INVERSE_DEPTH_SINGLE"
)

# location to save log files into
save_path_est="/home/cc/test/openvins_pra/sim_representations/algorithms"
save_path_gt="/home/cc/test/openvins_pra/sim_representations/truths"


#=============================================================
# Start Monte-Carlo Simulations
#=============================================================

# Loop through datasets
for m in "${!datasets[@]}"; do
# Loop through if use fej or not
for h in "${!usefej[@]}"; do
# Loop through all representations
for i in "${!representations[@]}"; do
# Monte Carlo runs for this dataset
for j in {00..09}; do

# start timing
start_time="$(date -u +%s)"

# filename change if we are using fej
if [ "${usefej[h]}" == "true" ]
then
    temp="_FEJ"
else
    temp=""
fi
filename_est="$save_path_est/${representations[i]}$temp/${datasets[m]}/estimate_$j.txt"
filename_gt="$save_path_gt/${datasets[m]}.txt"

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf simulation.launch \
  seed:="$((10#$j + 1))" \
  dataset:="${datasets[m]}.txt" \
  fej:="${usefej[h]}" \
  feat_rep:="${representations[i]}" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[m]} - ${usefej[h]} - ${representations[i]} - run $j took $elapsed seconds";


done
done
done
done
