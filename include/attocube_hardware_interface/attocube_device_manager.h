//
// Created by george on 10/19/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H

#include <ecc.h>

class AttocubeDeviceManager{
    AttocubeDeviceManager();
    ~AttocubeDeviceManager();

    int getDevicesAvailable();
    bool setupDevices();

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H
