# cmake_minimum_required(VERSION 3.3)
# project(butterworth)

# find_package(catkin QUIET COMPONENTS roscpp roslib sensor_msgs)
# catkin_package(CATKIN_DEPENDS roscpp roslib sensor_msgs)
# include_directories(src ${catkin_INCLUDE_DIRS})
# add_definitions(-std=c++17)
# #add_executable(butterworth_node src/butterworth.cpp)

# list(APPEND thirdparty_libraries
#         ${Boost_LIBRARIES}
#         ${OpenCV_LIBRARIES}
#         ${CERES_LIBRARIES}
#         ${catkin_LIBRARIES}
# )


# #target_link_libraries(butterworth_node ${thirdparty_libraries})
# #install(TARGETS butterworth_node
# #        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# #)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   nodelet
# )

# catkin_package(
# #  INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
# #  DEPENDS system_lib
# )

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# ## Declare a C++ library
# add_library(butterworth_nodelet
#   src/butterworth_nodelet.cpp
#   src/butterworth.cpp
# )
# target_link_libraries(butterworth_nodelet ${catkin_LIBRARIES})
# #if(catkin_EXPORTED_LIBRARIES)
# #  add_dependencies(butterworth_nodelet butterworth_node ${catkin_EXPORTED_LIBRARIES})
# #endif()
# install(TARGETS butterworth_nodelet
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
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
