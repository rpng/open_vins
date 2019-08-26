#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# config locations
modes=(
    "mono"
    "stereo"
)

# dataset locations
bagnames=(
    "V1_01_easy"
    "V1_02_medium"
    "V1_03_difficult"
    "V2_01_easy"
    "V2_02_medium"
    "V2_03_difficult"
)

# location to save log files into
save_path1="/home/patrick/github/pubs_data/pgeneva/2019_openvins/exp_testing/algorithms"
save_path2="/home/patrick/github/pubs_data/pgeneva/2019_openvins/exp_testing/timings"
bag_path="/home/patrick/datasets/eth"


#=============================================================
#=============================================================
#=============================================================




# Loop through all modes
for h in "${!modes[@]}"; do
# Loop through all datasets
for i in "${!bagnames[@]}"; do

# Monte Carlo runs for this dataset
for j in {00..00}; do

# start timing
start_time="$(date -u +%s)"
filename_est="$save_path1/ov_${modes[h]}/${bagnames[i]}/${start_time}_estimate.txt"
filename_time="$save_path2/ov_${modes[h]}/${bagnames[i]}/${start_time}_timing.txt"

# number of cameras
if [ "${modes[h]}" == "mono" ]
then
    temp="1"
else
    temp="2"
fi

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf pgeneva_eth_ros.launch max_cameras:="$temp" bag:="$bag_path/${bagnames[i]}.bag" dosave:="true" path_est:="$filename_est" path_time:="$filename_time" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${modes[h]} - ${bagnames[i]} - run $j took $elapsed seconds";

done


done
done


