//
// Created by george on 10/19/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H

#include <vector>
#include <ecc.h>
#include <rclcpp/rclcpp.hpp>

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

    /** @brief checks if the devices are initialised
     *  Checks the devices have been connected to and added to the devices_ list
     *  @return bool for if the devices are initialised
     */
     bool checkDevicesInitialised();

private:
    std::vector<int> devices_available_, devices_;

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_DEVICE_MANAGER_H
