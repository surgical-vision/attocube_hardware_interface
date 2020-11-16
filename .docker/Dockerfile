FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pip \
    build-essential \
    ros-noetic-rqt \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-rqt-console \
    ros-noetic-rqt-logger-level \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher \
    udev \
    git \
    ssh \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install catkin_tools

RUN rosdep update

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src/
COPY . /catkin_ws/src/attocube_hardware_interface
RUN git clone https://github.com/grdwyer/micro_resector_description.git

WORKDIR /catkin_ws/src/attocube_hardware_interface/.docker/EC100_install
# EC100 install commands
RUN cp nhands.rules /etc/udev/rules.d && \
    cp ecc.h /usr/include &&\
    cp libecc.so /usr/lib

RUN ldconfig

WORKDIR /catkin_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    catkin build"]

