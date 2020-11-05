//
// Created by george on 10/19/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H

#include <vector>
#include <ecc.h>
#include <ros/ros.h>

class AttocubeDeviceManager{
public:
    AttocubeDeviceManager();
    ~AttocubeDeviceManager();

    /** @brief checks the devices available
     *  Finds the devices connected by USB (and ethernet with the add on)
     * @return num_devices available
     */
    int getDevicesAvailable();

    /** @brief starts the connection to each device
     *
     * @return if each device was connected successfully
     */
    bool setupAllDevices();

private:
    std::vector<int> devices_available_, devices_;

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H
