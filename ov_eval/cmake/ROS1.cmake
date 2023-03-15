cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(catkin QUIET COMPONENTS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core)

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (catkin_FOUND AND ENABLE_ROS)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core
            INCLUDE_DIRS src/
            LIBRARIES ov_eval_lib
    )
else ()
    add_definitions(-DROS_AVAILABLE=0)
    message(WARNING "BUILDING WITHOUT ROS!")
    include(GNUInstallDirs)
    set(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    set(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
    set(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/open_vins/")
endif ()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

# If we are not building with ROS then we need to manually link to its headers
# This isn't that elegant of a way, but this at least allows for building without ROS
# See this stackoverflow answer: https://stackoverflow.com/a/11217008/7718197
if (NOT catkin_FOUND OR NOT ENABLE_ROS)

    message(STATUS "MANUALLY LINKING TO OV_CORE LIBRARY....")
    file(GLOB_RECURSE OVCORE_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_core/src/*.cpp")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_profile\\.cpp$")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_webcam\\.cpp$")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_tracking\\.cpp$")
    list(APPEND LIBRARY_SOURCES ${OVCORE_LIBRARY_SOURCES})
    include_directories(${CMAKE_SOURCE_DIR}/../ov_core/src/)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/../ov_core/src/
            DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

endif ()

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
target_link_libraries(ov_eval_lib ${thirdparty_libraries})
target_include_directories(ov_eval_lib PUBLIC src/)
install(TARGETS ov_eval_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

##################################################
# Make binary files!
##################################################

if (catkin_FOUND AND ENABLE_ROS)

    add_executable(pose_to_file src/pose_to_file.cpp)
    target_link_libraries(pose_to_file ov_eval_lib ${thirdparty_libraries})
    install(TARGETS pose_to_file
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    add_executable(live_align_trajectory src/live_align_trajectory.cpp)
    target_link_libraries(live_align_trajectory ov_eval_lib ${thirdparty_libraries})
    install(TARGETS live_align_trajectory
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    catkin_install_python(PROGRAMS python/pid_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    catkin_install_python(PROGRAMS python/pid_sys.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

endif ()

add_executable(format_converter src/format_converter.cpp)
target_link_libraries(format_converter ov_eval_lib ${thirdparty_libraries})
install(TARGETS format_converter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(error_comparison src/error_comparison.cpp)
target_link_libraries(error_comparison ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_comparison
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(error_dataset src/error_dataset.cpp)
target_link_libraries(error_dataset ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_dataset
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(error_singlerun src/error_singlerun.cpp)
target_link_libraries(error_singlerun ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_singlerun
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(error_simulation src/error_simulation.cpp)
target_link_libraries(error_simulation ov_eval_lib ${thirdparty_libraries})
install(TARGETS error_simulation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(timing_comparison src/timing_comparison.cpp)
target_link_libraries(timing_comparison ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_comparison
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(timing_flamegraph src/timing_flamegraph.cpp)
target_link_libraries(timing_flamegraph ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_flamegraph
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(timing_histogram src/timing_histogram.cpp)
target_link_libraries(timing_histogram ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_histogram
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(timing_percentages src/timing_percentages.cpp)
target_link_libraries(timing_percentages ov_eval_lib ${thirdparty_libraries})
install(TARGETS timing_percentages
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(plot_trajectories src/plot_trajectories.cpp)
target_link_libraries(plot_trajectories ov_eval_lib ${thirdparty_libraries})
install(TARGETS plot_trajectories
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


