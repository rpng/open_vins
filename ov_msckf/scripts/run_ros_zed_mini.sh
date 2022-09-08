#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# estimator configurations
modes=(
    "stereo"
)

# dataset locations
bagnames=(
    "fan_no_prop_no_circle" # bag needs to end early as there is a hard landing....
    
)

# what sensor configuration each dataset has
config=(
    "zed_mini" # bag needs to end early as there is a hard landing....
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
# indoor forward
    "12" # bag needs to end early as there is a hard landing....
)

# location to save log files into
save_path1="/home/chenyu/Music/ov_result/"
save_path2="/home/chenyu/Music/ov_result/timings"
bag_path="/home/chenyu/Music/catkin_ws_ov/data"
ov_ver="2.6.2"



#=============================================================
#=============================================================
#=============================================================

# Loop through all datasets
for i in "${!bagnames[@]}"; do
# Loop through all modes
for h in "${!modes[@]}"; do
for k in "${!config[@]}"; do

# Monte Carlo runs for this dataset
# If you want more runs, change the below loop
for j in {00..03}; do

# start timing
start_time="$(date -u +%s)"
filename_est="$save_path1/ov_${modes[h]}/${config[k]}/${bagnames[i]}/${j}_estimate.txt"
# filename_time="$save_path2/ov_${modes[h]}/${config[k]}/${bagnames[i]}/${j}_timing.txt"

# number of cameras

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
done
