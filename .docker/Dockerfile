FROM ros:melodic

RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    ros-melodic-rqt \
    ros-melodic-moveit \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    udev

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
RUN ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash && \
    catkin build"]

