cmake_minimum_required(VERSION 3.10)
project(conformal_stage1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS ov_core ov_init ov_msckf)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)

catkin_package()

add_executable(run_asl_msckf
  run_asl_msckf.cpp
  DiagnosticsLogger.cpp
)

target_include_directories(run_asl_msckf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(run_asl_msckf PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
)

add_dependencies(run_asl_msckf ${catkin_EXPORTED_TARGETS})

install(TARGETS run_asl_msckf
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
