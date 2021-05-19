FROM ros:noetic-robot

RUN apt-get update && apt-get install -y libeigen3-dev git cmake ninja-build clang

RUN git clone https://github.com/opencv/opencv/
RUN git clone https://github.com/opencv/opencv_contrib/
RUN cmake -D OPENCV_EXTRA_MODULES_PATH=opencv_contrib/modules -S opencv -B opencv/build -G Ninja
RUN sudo ninja -C opencv/build install

RUN sudo apt-get install -y python3-catkin-tools python3-osrf-pycommon python3-catkin-pkg
