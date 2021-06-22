#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# estimator configurations
modes=(
  "mono"
  "binocular"
  "stereo"
)

# dataset locations
bagnames=(
  "dataset-room1_512_16"
  "dataset-room2_512_16"
  "dataset-room3_512_16"
  "dataset-room4_512_16"
  "dataset-room5_512_16"
  "dataset-room6_512_16"
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
  "0"
  "0"
  "0"
  "0"
  "0"
  "0"
)

# threshold for variance to detect if the unit has moved yet
# these datasets seem to have very large variablity in their starts
imuthreshold=(
  "0.60"
  "0.60"
  "0.60"
  "0.60"
  "0.60"
  "0.45"
)

# location to save log files into
save_path1="/home/patrick/github/pubs_data/pgeneva/2020_openvins_2.3.1/exp_tum/algorithms"
bag_path="/media/patrick/RPNG\ FLASH\ 2/tumvi"


#=============================================================
#=============================================================
#=============================================================


# Loop through all modes
for h in "${!modes[@]}"; do
# Loop through all datasets
for i in "${!bagnames[@]}"; do

# Monte Carlo runs for this dataset
# If you want more runs, change the below loop
for j in {00..04}; do

# start timing
start_time="$(date -u +%s)"
filename_est="$save_path1/ov_2.3.1_${modes[h]}/${bagnames[i]}/${start_time}_estimate.txt"

# number of cameras
if [ "${modes[h]}" == "mono" ]
then
  temp1="1"
  temp2="true"
fi
if [ "${modes[h]}" == "binocular" ]
then
  temp1="2"
  temp2="false"
fi
if [ "${modes[h]}" == "stereo" ]
then
  temp1="2"
  temp2="true"
fi

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf pgeneva_ros_tum.launch \
  max_cameras:="$temp1" \
  use_stereo:="$temp2" \
  bag:="$bag_path/${bagnames[i]}.bag" \
  bag_start:="${bagstarttimes[i]}" \
  init_imu_thresh:="${imuthreshold[i]}" \
  dosave:="true" \
  path_est:="$filename_est" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${modes[h]} - ${bagnames[i]} - run $j took $elapsed seconds";

done


done
done


