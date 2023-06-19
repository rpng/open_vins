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

# estimator configurations
usefej=(
    "true"
#    "false"
)

# feature representations
representations=(
    "GLOBAL_3D"
#    "GLOBAL_FULL_INVERSE_DEPTH"
    "ANCHORED_3D"
    "ANCHORED_FULL_INVERSE_DEPTH"
    "ANCHORED_MSCKF_INVERSE_DEPTH"
#    "ANCHORED_INVERSE_DEPTH_SINGLE"
)

# extra configuration (feature scene depth)
configs=(
    "03m_"
    "06m_"
    "10m_"
)
configs_params=(
    "feat_dist_min:=2.0 feat_dist_max:=4.0"
    "feat_dist_min:=5.0 feat_dist_max:=7.0"
    "feat_dist_min:=9.0 feat_dist_max:=11.0"
)

# location to save log files into
save_path1="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_featrep/algorithms"
save_path2="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_featrep/timings"
save_path3="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/sim_featrep/truths"


#=============================================================
# Start Monte-Carlo Simulations
#=============================================================

big_start_time="$(date -u +%s)"

# Loop through datasets
for h in "${!datasets[@]}"; do
# Loop through if use fej or not
for m in "${!usefej[@]}"; do
# Loop through all representations
for k in "${!configs[@]}"; do
for i in "${!representations[@]}"; do
# Monte Carlo runs for this dataset
for j in {00..49}; do

# start timing
start_time="$(date -u +%s)"

# filename change if we are using fej
temp=""
if [ "${usefej[m]}" == "true" ]
then
    temp="FEJ_"
fi
folder="${temp}${configs[k]}${representations[i]}"
filename_est="$save_path1/$folder/${datasets[h]}/estimate_$j.txt"
filename_gt="$save_path3/${datasets[h]}.txt"

# launch the simulation script
roslaunch ov_msckf simulation.launch \
  verbosity:="WARNING" \
  seed:="$((10#$j + 1))" \
  dataset:="${datasets[h]}.txt" \
  max_cameras:="1" \
  fej:="${usefej[m]}" \
  feat_rep:="${representations[i]}" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" \
  ${configs_params[k]} &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - ${folder} - run $j took $elapsed seconds";


done
done
done
done
done



# print out the time elapsed
big_end_time="$(date -u +%s)"
big_elapsed="$(($big_end_time-$big_start_time))"
echo "BASH: script took $big_elapsed seconds in total!!";


