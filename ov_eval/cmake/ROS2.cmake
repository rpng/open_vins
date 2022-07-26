cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ov_core REQUIRED)

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (NOT ENABLE_ROS)
    message(FATAL_ERROR "Build with ROS1.cmake if you don't have ROS.")
endif ()
add_definitions(-DROS_AVAILABLE=2)

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
)

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/alignment/AlignTrajectory.cpp
        src/alignment/AlignUtils.cpp
        src/calc/ResultTrajectory.cpp
        src/calc/ResultSimulation.cpp
        src/utils/Loader.cpp
)
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_eval_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
ament_target_dependencies(ov_eval_lib rclcpp ov_core)
target_link_libraries(ov_eval_lib ${thirdparty_libraries})
target_include_directories(ov_eval_lib PUBLIC src/)
install(TARGETS ov_eval_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(ov_eval_lib)

##################################################
# Make binary files!
##################################################

# TODO: UPGRADE THIS TO ROS2 AS ANOTHER FILE!!
#if (catkin_FOUND AND ENABLE_ROS)
#    add_executable(pose_to_file src/pose_to_file.cpp)
#    target_link_libraries(pose_to_file ov_eval_lib ${thirdparty_libraries})
#    add_executable(live_align_trajectory src/live_align_trajectory.cpp)
#    target_link_libraries(live_align_trajectory ov_eval_lib ${thirdparty_libraries})
#endif ()

add_executable(format_converter src/format_converter.cpp)
ament_target_dependencies(format_converter rclcpp ov_core)
target_link_libraries(format_converter ov_eval_lib ${thirdparty_libraries})
install(TARGETS format_converter DESTINATION lib/${PROJECT_NAME})

add_executable(error_comparison src/error_comparison.cpp)
ament_target_dependencies(error_comparison rclcpp ov_core)
target_link_libraries(error_comparison ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_comparison DESTINATION lib/${PROJECT_NAME})

add_executable(error_dataset src/error_dataset.cpp)
ament_target_dependencies(error_dataset rclcpp ov_core)
target_link_libraries(error_dataset ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_dataset DESTINATION lib/${PROJECT_NAME})

add_executable(error_singlerun src/error_singlerun.cpp)
ament_target_dependencies(error_singlerun rclcpp ov_core)
target_link_libraries(error_singlerun ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_singlerun DESTINATION lib/${PROJECT_NAME})

add_executable(error_simulation src/error_simulation.cpp)
ament_target_dependencies(error_simulation rclcpp ov_core)
target_link_libraries(error_simulation ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_simulation DESTINATION lib/${PROJECT_NAME})

add_executable(timing_comparison src/timing_comparison.cpp)
ament_target_dependencies(timing_comparison rclcpp ov_core)
target_link_libraries(timing_comparison ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_comparison DESTINATION lib/${PROJECT_NAME})

add_executable(timing_flamegraph src/timing_flamegraph.cpp)
ament_target_dependencies(timing_flamegraph rclcpp ov_core)
target_link_libraries(timing_flamegraph ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_flamegraph DESTINATION lib/${PROJECT_NAME})

add_executable(timing_histogram src/timing_histogram.cpp)
ament_target_dependencies(timing_histogram rclcpp ov_core)
target_link_libraries(timing_histogram ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_histogram DESTINATION lib/${PROJECT_NAME})

add_executable(timing_percentages src/timing_percentages.cpp)
ament_target_dependencies(timing_percentages rclcpp ov_core)
target_link_libraries(timing_percentages ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_percentages DESTINATION lib/${PROJECT_NAME})

add_executable(plot_trajectories src/plot_trajectories.cpp)
ament_target_dependencies(plot_trajectories rclcpp ov_core)
target_link_libraries(plot_trajectories ov_eval_lib ${thirdparty_libraries})
install(TARGETS plot_trajectories DESTINATION lib/${PROJECT_NAME})


##################################################
# Python scripts!
##################################################

# TODO: UPGRADE THIS TO ROS2 AS ANOTHER FILE!!
#if (catkin_FOUND AND ENABLE_ROS)
#    catkin_install_python(PROGRAMS python/pid_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#    catkin_install_python(PROGRAMS python/pid_sys.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#endif ()


# finally define this as the package
ament_package()