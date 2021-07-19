FROM osrf/ros:noetic-desktop-full

# =========================================================
# =========================================================

# Are you are looking for how to use this docker file?
#   - https://docs.openvins.com/dev-docker.html
#   - https://docs.docker.com/get-started/
#   - http://wiki.ros.org/docker/Tutorials/Docker

# =========================================================
# =========================================================

# Dependencies we use, catkin tools is very good build system
# Also some helper utitiles for fast in terminal edits (nano etc)
RUN apt-get update && apt-get install -y libeigen3-dev nano git cmake
RUN sudo apt-get install -y python3-catkin-tools python3-osrf-pycommon

# Install deps needed for clion remote debugging
# https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/
RUN apt-get update && apt-get install -y ssh build-essential gcc g++ \
    gdb clang cmake rsync tar python && apt-get clean
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd
RUN useradd -m user && yes password | passwd user
RUN usermod -s /bin/bash user
CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]

