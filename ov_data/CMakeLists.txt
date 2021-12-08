cmake_minimum_required(VERSION 3.12)

# Project name
project(ov_data)

# Find our ROS version!
# NOTE: Default to using the ROS1 package if both are in our enviroment
# NOTE: https://github.com/romainreignier/share_ros1_ros2_lib_demo
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if(catkin_FOUND)
    message(STATUS "ROS *1* version found")
    catkin_package()
elseif(ament_cmake_FOUND)
    message(STATUS "ROS *2* version found")
    ament_package()
else()
    message(STATUS "No ROS versions found, doing nothing...")
endif()
