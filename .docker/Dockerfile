FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    ros-noetic-rqt \
    ros-noetic-moveit \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    udev \
    git \
    ssh \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

RUN mkdir -p /catkin_ws/src
COPY . /catkin_ws/src/attocube_hardware_interface

WORKDIR /catkin_ws/src/attocube_hardware_interface/.docker/EC100_install
# EC100 install commands
RUN cp nhands.rules /etc/udev/rules.d && \
    cp ecc.h /usr/include &&\
    cp libecc.so /usr/lib

RUN ldconfig

WORKDIR /catkin_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    catkin_make"]