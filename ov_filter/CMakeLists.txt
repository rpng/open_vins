cmake_minimum_required(VERSION 3.3)
project(butterworth)

find_package(catkin QUIET COMPONENTS roscpp roslib sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp roslib sensor_msgs)
include_directories(src ${catkin_INCLUDE_DIRS})
add_definitions(-std=c++17)
add_executable(butterworth_node src/butterworth.cpp)

list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
)


target_link_libraries(butterworth_node ${thirdparty_libraries})
install(TARGETS butterworth_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

