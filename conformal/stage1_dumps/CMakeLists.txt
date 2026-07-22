# CMakeLists.txt -- standalone build for the conformal Stage-1 runner.
#
# Companion to: Section 18.1 (ROS-free OpenVINS build) and 18.2 (run_asl_msckf.cpp).
#
# WHY STANDALONE: the paper's central defensibility claim (Section 18.3 / 4.4) is that
# ov_msckf/ and ov_core/ stay byte-for-byte unmodified. So instead of editing
# ov_msckf/CMakeLists.txt to add a target, we build the runner here and link against the
# OpenVINS libraries produced by the normal ROS-free build:
#     cd ov_msckf && mkdir build && cd build && cmake -DENABLE_ROS=OFF .. && make -j
#
# BUILD:
#     cd conformal/stage1_dumps && mkdir build && cd build
#     cmake -DOPENVINS_ROOT=../../.. ..
#     make -j
#
# TODO(intern): OpenVINS' CMake is catkin/colcon-oriented; the cleanest path is to
# `find_package(ov_msckf)`/`find_package(ov_core)` after a local `make install`, or point
# the INCLUDE/LINK dirs below at your ov_msckf/build tree. Adjust to your setup.

cmake_minimum_required(VERSION 3.10)
project(conformal_stage1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPENVINS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "OpenVINS repo root")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
# TODO(intern): add HDF5 once you pick a binding (HighFive is header-only over libhdf5):
#   find_package(HDF5 REQUIRED COMPONENTS CXX)
#   find_package(HighFive REQUIRED)

add_executable(run_asl_msckf
  run_asl_msckf.cpp
  # DiagnosticsLogger.hpp is header-only for now; add DiagnosticsLogger.cpp if you split it.
)

target_include_directories(run_asl_msckf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OPENVINS_ROOT}/ov_msckf/src
  ${OPENVINS_ROOT}/ov_core/src
  ${OPENVINS_ROOT}/ov_init/src
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# TODO(intern): point these at the libraries emitted by the ov_msckf ROS-free build.
target_link_libraries(run_asl_msckf PRIVATE
  # ov_msckf_lib ov_core_lib ov_init_lib   # <- names from your build tree
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Boost::filesystem
  Boost::system
  # ${HDF5_CXX_LIBRARIES}
)
