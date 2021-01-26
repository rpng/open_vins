cmake_minimum_required(VERSION 2.8.8)

# Project name
project(ov_eval)

# Find catkin (the ROS build system)
find_package(catkin QUIET COMPONENTS roscpp rospy geometry_msgs nav_msgs sensor_msgs)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
#message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# check if we have our python libs files
# sudo apt-get install python-matplotlib python-numpy python2.7-dev
find_package(PythonLibs 2.7)
if(PYTHONLIBS_FOUND AND NOT DISABLE_MATPLOTLIB)
    add_definitions(-DHAVE_PYTHONLIBS=1)
    message(STATUS "PYTHON VERSION: " ${PYTHONLIBS_VERSION_STRING})
    message(STATUS "PYTHON INCLUDE: " ${PYTHON_INCLUDE_DIRS})
    message(STATUS "PYTHON LIBRARIES: " ${PYTHON_LIBRARIES})
endif()

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs sensor_msgs
            INCLUDE_DIRS src
            LIBRARIES ov_eval_lib
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        /usr/local/lib/python2.7/dist-packages/numpy/core/include
        /usr/local/lib/python2.7/site-packages/numpy/core/include
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${catkin_LIBRARIES}
)

##################################################
# Make the core library
##################################################
add_library(ov_eval_lib
        src/dummy.cpp
        src/alignment/AlignTrajectory.cpp
        src/alignment/AlignUtils.cpp
        src/calc/ResultTrajectory.cpp
        src/calc/ResultSimulation.cpp
        src/utils/Loader.cpp
)
target_link_libraries(ov_eval_lib ${thirdparty_libraries})


##################################################
# Make binary files!
##################################################


if (catkin_FOUND)
    add_executable(pose_to_file src/pose_to_file.cpp)
    target_link_libraries(pose_to_file ov_eval_lib ${thirdparty_libraries})
endif()


add_executable(format_converter src/format_converter.cpp)
target_link_libraries(format_converter ov_eval_lib ${thirdparty_libraries})

add_executable(error_comparison src/error_comparison.cpp)
target_link_libraries(error_comparison ov_eval_lib ${thirdparty_libraries})

add_executable(error_dataset src/error_dataset.cpp)
target_link_libraries(error_dataset ov_eval_lib ${thirdparty_libraries})

add_executable(error_singlerun src/error_singlerun.cpp)
target_link_libraries(error_singlerun ov_eval_lib ${thirdparty_libraries})

add_executable(error_simulation src/error_simulation.cpp)
target_link_libraries(error_simulation ov_eval_lib ${thirdparty_libraries})

add_executable(timing_flamegraph src/timing_flamegraph.cpp)
target_link_libraries(timing_flamegraph ov_eval_lib ${thirdparty_libraries})

add_executable(timing_comparison src/timing_comparison.cpp)
target_link_libraries(timing_comparison ov_eval_lib ${thirdparty_libraries})

add_executable(timing_percentages src/timing_percentages.cpp)
target_link_libraries(timing_percentages ov_eval_lib ${thirdparty_libraries})

add_executable(plot_trajectories src/plot_trajectories.cpp)
target_link_libraries(plot_trajectories ov_eval_lib ${thirdparty_libraries})


##################################################
# Python scripts!
##################################################
if (catkin_FOUND)
    catkin_install_python(PROGRAMS python/pid_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    catkin_install_python(PROGRAMS python/pid_sys.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()




