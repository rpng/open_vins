FROM ros:noetic-robot

RUN apt-get update && apt-get install -y libeigen3-dev git cmake ninja-build clang build-essential python3-catkin-tools python3-osrf-pycommon python3-catkin-pkg

RUN git clone https://github.com/opencv/opencv/
RUN git clone https://github.com/opencv/opencv_contrib/
RUN cmake -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -S opencv -B opencv/build -G Ninja
RUN sudo ninja -C opencv/build install

RUN sh -c 'echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc'

ENTRYPOINT /bin/bash

# docker build . --tag open_vins
# docker run --rm --interactive --tty --volume "${PWD}":/workspace/src/open_vins -w /workspace/ open_vins
