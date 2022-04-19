#!/bin/bash

cd /dev_ws/src/attocube_hardware_interface/.docker/EC100_install
cp nhands.rules /etc/udev/rules.d && \
cp ecc.h /usr/include &&\
cp libecc.so /usr/lib

ldconfig