FROM ros:foxy

RUN apt-get update && apt-get install -y \
    software-properties-common \
    ssh \
    git \
    udev \
    usbutils \
    libboost-dev \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    ros-foxy-xacro \
    wget \
    ros-foxy-rqt \
    ros-foxy-rviz2 \
    ros-foxy-angles \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /dev_ws/src
WORKDIR /dev_ws/src/
COPY . /dev_ws/src/attocube_hardware_interface
RUN git clone https://github.com/grdwyer/micro_resector_description.git

WORKDIR /dev_ws/src/attocube_hardware_interface/.docker/EC100_install
# EC100 install commands
RUN cp nhands.rules /etc/udev/rules.d && \
    cp ecc.h /usr/include &&\
    cp libecc.so /usr/lib

RUN ldconfig

WORKDIR /dev_ws/
# RUN ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && \
#     colcon build"]

RUN mv /dev_ws/src/attocube_hardware_interface/.docker/attocube_entrypoint.sh /attocube_entrypoint.sh && \
    mv /dev_ws/src/attocube_hardware_interface/.docker/nigel_start.sh /nigel_start.sh && \
    chmod +x /nigel_start.sh && \
    chmod +x /attocube_entrypoint.sh

# ENTRYPOINT [ "/attocube_entrypoint.sh" ]
# CMD ["/bin/bash", "-c", "rosrun attocube_hardware_interface hardware_interface" ]
CMD ["/bin/bash", "-c", "bash" ]



