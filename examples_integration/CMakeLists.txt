cmake_minimum_required(VERSION 3.5)
project(examples_integration_imt_bcom)

# Compiler en C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trouver OpenVINS (si installé globalement)
find_library(OV_MSCKF_LIB ov_msckf_lib PATHS /usr/local/lib)

# Include directories
include_directories(
    /usr/local/include/open_vins
    /usr/include/eigen3
)

# Trouver OpenCV
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Exemple minimal
add_executable(minimal_vio_example minimal_vio_example.cpp)

target_link_libraries(minimal_vio_example
    ${OV_MSCKF_LIB}
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)

# Exemple lecteur dataset EuRoC
add_executable(euroc_reader_example euroc_reader_example.cpp)

target_link_libraries(euroc_reader_example
    ${OV_MSCKF_LIB}
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)

install(TARGETS minimal_vio_example euroc_reader_example
    RUNTIME DESTINATION bin
)
