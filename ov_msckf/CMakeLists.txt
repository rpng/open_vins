cmake_minimum_required(VERSION 2.8.8)

# Project name
project(ov_msckf)

# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core)

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED)
endif()
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
#message(WARNING "EIGEN VERSION: " ${EIGEN3_VERSION})
message(WARNING "OPENCV VERSION: " ${OpenCV_VERSION})
message(WARNING "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core
            INCLUDE_DIRS src
            LIBRARIES ov_msckf_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

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
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)

# If we are not building with ROS then we need to manually link to its headers
# This isn't that elegant of a way, but this at least allows for building without ROS
# See this stackoverflow answer: https://stackoverflow.com/a/11217008/7718197
if (NOT catkin_FOUND)
    message(WARNING "MANUALLY LINKING TO OV_CORE LIBRARY....")
    include_directories(${ov_core_SOURCE_DIR}/src/)
    list(APPEND thirdparty_libraries ov_core_lib)
endif()

##################################################
# Make the core library
##################################################
list(APPEND library_source_files
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/update/UpdaterZeroVelocity.cpp
)
if (catkin_FOUND)
    list(APPEND library_source_files
        src/core/RosVisualizer.cpp
    )
endif()
add_library(ov_msckf_lib SHARED ${library_source_files})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src)


##################################################
# Make binary files!
##################################################


if (catkin_FOUND)

    add_executable(run_serial_msckf src/ros_serial_msckf.cpp)
    target_link_libraries(run_serial_msckf ov_msckf_lib ${thirdparty_libraries})

    add_executable(run_subscribe_msckf src/ros_subscribe_msckf.cpp)
    target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})

    add_executable(run_simulation src/run_simulation.cpp)
    target_link_libraries(run_simulation ov_msckf_lib ${thirdparty_libraries})

else()

    add_executable(run_simulation src/run_simulation.cpp)
    target_link_libraries(run_simulation ov_msckf_lib ${thirdparty_libraries})

endif()


add_executable(test_sim_meas src/test_sim_meas.cpp)
target_link_libraries(test_sim_meas ov_msckf_lib ${thirdparty_libraries})

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
target_link_libraries(test_sim_repeat ov_msckf_lib ${thirdparty_libraries})


