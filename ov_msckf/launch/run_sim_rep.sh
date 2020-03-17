#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# config locations
usefej=(
    "false"
    "true"
)

# dataset locations
representations=(
#    "GLOBAL_3D"
#    "GLOBAL_FULL_INVERSE_DEPTH"
#    "ANCHORED_3D"
#    "ANCHORED_FULL_INVERSE_DEPTH"
    "ANCHORED_MSCKF_INVERSE_DEPTH"
    "ANCHORED_INVERSE_DEPTH_SINGLE"
)


# location to save log files into
save_path="/home/patrick/github/pubs_data/pgeneva/2020_openvins/sim_representations/algorithms"


#=============================================================
#=============================================================
#=============================================================


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
filename="$save_path/${representations[i]}$temp/udel_gore/estimate_$j.txt"

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_msckf pgeneva_sim.launch seed:="$j" fej:="${usefej[h]}" feat_rep:="${representations[i]}" num_clones:="6" num_slam:="50" num_pts:="50" dosave:="true" path_est:="$filename" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${usefej[h]} - ${representations[i]} - run $j took $elapsed seconds";


done



done
done

