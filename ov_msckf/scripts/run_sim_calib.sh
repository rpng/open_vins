#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# dataset locations
datasets=(
    "udel_gore"
#    "udel_arl"
#    "tum_corridor1"
#    "udel_neighborhood"
)



# location to save log files into
save_path_est="/home/patrick/github/pubs_data/pgeneva/2020_openvins/NEW_sim_calibration/algorithms"
save_path_gt="/home/patrick/github/pubs_data/pgeneva/2020_openvins/NEW_sim_calibration/truths"


#=============================================================
#=============================================================
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
roslaunch ov_msckf pgeneva_sim.launch seed:="$j" dataset:="${datasets[h]}.txt" sim_do_calibration:="false" sim_do_perturbation:="false" max_cameras:="1" num_clones:="11" num_slam:="50" num_pts:="100" dosave_pose:="true" path_est:="$filename_est" path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - static_groundtruth - run $j took $elapsed seconds";
#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_calib_groundtruth/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf pgeneva_sim.launch seed:="$j" dataset:="${datasets[h]}.txt" sim_do_calibration:="true" sim_do_perturbation:="false" max_cameras:="1" num_clones:="11" num_slam:="50" num_pts:="100" dosave_pose:="true" path_est:="$filename_est" path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - calib_groundtruth - run $j took $elapsed seconds";
#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_calib_perturbed/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf pgeneva_sim.launch seed:="$j" dataset:="${datasets[h]}.txt" sim_do_calibration:="true" sim_do_perturbation:="true" max_cameras:="1" num_clones:="11" num_slam:="50" num_pts:="100" dosave_pose:="true" path_est:="$filename_est" path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - calib_perturbed - run $j took $elapsed seconds";
#===============================================
#===============================================
start_time="$(date -u +%s)"
filename_est="$save_path_est/ov_23_static_perturbed/${datasets[h]}/estimate_$j.txt"
roslaunch ov_msckf pgeneva_sim.launch seed:="$j" dataset:="${datasets[h]}.txt" sim_do_calibration:="false" sim_do_perturbation:="true" max_cameras:="1" num_clones:="11" num_slam:="50" num_pts:="100" dosave_pose:="true" path_est:="$filename_est" path_gt:="$filename_gt" &> /dev/null
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[h]} - static_perturbed - run $j took $elapsed seconds";
#===============================================
#===============================================




done
done

