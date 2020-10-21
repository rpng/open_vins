#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# dataset locations
datasets=(
    "udel_gore"
    "udel_arl"
#    "tum_corridor1"
#    "udel_neighborhood"
)

# number of cameras
cameras=(
    "1"
    "2"
    "3"
    "4"
)


# location to save log files into
save_path_est="/home/patrick/github/pubs_data/pgeneva/2020_openvins/NEW_sim_cameras/algorithms"
save_path_gt="/home/patrick/github/pubs_data/pgeneva/2020_openvins/NEW_sim_cameras/truths"


#=============================================================
#=============================================================
#=============================================================


# Loop through if use fej or not
for h in "${!datasets[@]}"; do
# Loop through all representations
for i in "${!cameras[@]}"; do

# Monte Carlo runs for this dataset
for j in {00..02}; do

# start timing
start_time="$(date -u +%s)"

# our filename
filename_est="$save_path_est/ov_v23_cam${cameras[i]}/${datasets[h]}/estimate_$j.txt"
filename_gt="$save_path_gt/${datasets[h]}.txt"

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf pgeneva_sim.launch seed:="$j" max_cameras:="${cameras[i]}" dataset:="${datasets[h]}.txt" num_clones:="11" num_slam:="50" num_pts:="100" dosave_pose:="true" path_est:="$filename_est" path_gt:="$filename_gt" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - ${cameras[i]} - run $j took $elapsed seconds";


done



done
done

