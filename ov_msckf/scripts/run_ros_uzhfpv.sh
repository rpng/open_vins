#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

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
    "indoor_forward_3_snapdragon_with_gt" # bag needs to end early as there is a hard landing....
    "indoor_forward_5_snapdragon_with_gt"
    "indoor_forward_6_snapdragon_with_gt"
    "indoor_forward_7_snapdragon_with_gt"
    "indoor_forward_9_snapdragon_with_gt"
    "indoor_forward_10_snapdragon_with_gt"
    "indoor_45_2_snapdragon_with_gt"
    "indoor_45_4_snapdragon_with_gt"
#    "indoor_45_9_snapdragon_with_gt" # problem one, seems to fail part way in due to freefalling?
    "indoor_45_12_snapdragon_with_gt"
    "indoor_45_13_snapdragon_with_gt"
    "indoor_45_14_snapdragon_with_gt"
    "outdoor_forward_1_snapdragon_with_gt"
    "outdoor_forward_3_snapdragon_with_gt"
    "outdoor_forward_5_snapdragon_with_gt"
    "outdoor_45_1_snapdragon_with_gt"
)

# what sensor configuration each dataset has
config=(
    "uzhfpv_indoor" # bag needs to end early as there is a hard landing....
    "uzhfpv_indoor"
    "uzhfpv_indoor"
    "uzhfpv_indoor"
    "uzhfpv_indoor"
    "uzhfpv_indoor"
    "uzhfpv_indoor_45"
    "uzhfpv_indoor_45"
#    "uzhfpv_indoor_45" # problem one, seems to fail part way in due to freefalling?
    "uzhfpv_indoor_45"
    "uzhfpv_indoor_45"
    "uzhfpv_indoor_45"
    "uzhfpv_outdoor"
    "uzhfpv_outdoor"
    "uzhfpv_outdoor"
    "uzhfpv_outdoor_45"
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
# indoor forward
    "0" # bag needs to end early as there is a hard landing....
    "0"
    "0"
    "0"
    "0"
    "0"
# indoor 45 degree
    "0"
    "0"
#    "0" # problem one, seems to fail part way in due to freefalling?
    "0"
    "0"
    "0"
# outdoor forward and 45
    "0"
    "0"
    "0"
    "0"
)

# location to save log files into
save_path1="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/exp_uzhfpv/algorithms"
save_path2="/home/patrick/github/pubs_data/pgeneva/2023_openvins_reproduce/exp_uzhfpv/timings"
bag_path="/media/patrick/RPNG FLASH 3/"
ov_ver="2.7"


#=============================================================
#=============================================================
#=============================================================

big_start_time="$(date -u +%s)"

# Loop through all datasets
for i in "${!bagnames[@]}"; do
# Loop through all modes
for h in "${!modes[@]}"; do

# Monte Carlo runs for this dataset
# If you want more runs, change the below loop
for j in {00..00}; do

# start timing
start_time="$(date -u +%s)"
filename_est="$save_path1/ov_${ov_ver}_${modes[h]}/${bagnames[i]}/${j}_estimate.txt"
filename_time="$save_path2/ov_${ov_ver}_${modes[h]}/${bagnames[i]}/${j}_timing.txt"

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
# subscribe=live pub, serial=read from bag (fast)
roslaunch ov_msckf serial.launch \
  max_cameras:="$temp1" \
  use_stereo:="$temp2" \
  config:="${config[i]}" \
  dataset:="${bagnames[i]}" \
  bag:="$bag_path/${config[i]}/${bagnames[i]}.bag" \
  bag_start:="${bagstarttimes[i]}" \
  dobag:="true" \
  dosave:="true" \
  path_est:="$filename_est" \
  dotime:="true" \
  dolivetraj:="true" \
  path_time:="$filename_time" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${modes[h]} - ${bagnames[i]} - run $j took $elapsed seconds";

done


done
done



# print out the time elapsed
big_end_time="$(date -u +%s)"
big_elapsed="$(($big_end_time-$big_start_time))"
echo "BASH: script took $big_elapsed seconds in total!!";


