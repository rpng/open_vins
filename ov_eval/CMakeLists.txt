cmake_minimum_required(VERSION 2.8.8)

# Project name
project(ov_eval)

# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
catkin_package(
        CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs
        INCLUDE_DIRS src
        LIBRARIES ov_eval_lib
)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

##################################################
# Make the core library
##################################################
add_library(ov_eval_lib
        src/dummy.cpp
        src/alignment/AlignTrajectory.cpp
        src/alignment/AlignUtils.cpp
        src/utils/Recorder.h
)
target_link_libraries(ov_eval_lib ${thirdparty_libraries})


##################################################
# Make binary files!
##################################################
add_executable(pose_to_file src/pose_to_file.cpp)
target_link_libraries(pose_to_file ov_eval_lib ${thirdparty_libraries})


add_executable(format_converter src/format_converter.cpp)
target_link_libraries(format_converter ov_eval_lib ${thirdparty_libraries})


