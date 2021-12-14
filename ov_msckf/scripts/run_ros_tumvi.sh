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

# location to save log files into
save_path1="/home/chuchu/test_ov/openvins_pra/exp_tum/algorithms"
save_path2="/home/chuchu/test_ov/openvins_pra/exp_tum/timings"
bag_path="/home/chuchu/datasets/tum_vi"


#=============================================================
#=============================================================
#=============================================================
# TODO: Still need to test all, see how to get imu threshold in
# Loop through all modes
for h in "${!modes[@]}"; do
# Loop through all datasets
for i in "${!bagnames[@]}"; do

# Monte Carlo runs for this dataset
# If you want more runs, change the below loop
for j in {00..04}; do

# start timing
start_time="$(date -u +%s)"
filename_est="$save_path1/ov_2.4_${modes[h]}/${bagnames[i]}/${j}_estimate.txt"
filename_time="$save_path2/ov_2.4_${modes[h]}/${bagnames[i]}/${j}_timing.txt"

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
roslaunch ov_msckf serial.launch \
  max_cameras:="$temp1" \
  use_stereo:="$temp2" \
  config:="tum_vi" \
  bag:="$bag_path/${bagnames[i]}.bag" \
  bag_start:="${bagstarttimes[i]}" \
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


