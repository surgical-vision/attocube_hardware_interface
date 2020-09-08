//
// Created by george on 9/3/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H

#include <string>
#include <vector>
#include <attocube_hardware_interface/attocube_actors.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>
#include <ecc.h>

class AttocubeHardwareInterface {
    AttocubeHardwareInterface(ros::NodeHandle& nh);
    ~AttocubeHardwareInterface();
    void getConfigFromParam(); // Setup the number of controller and the actor settings
    void setupDevices();
    void readPositions();
    void writePositions();
    void getDevicesAvailable();
    void getActorFromName(std::string& joint_name, int& device, int& axis);

    ros::NodeHandle nh_;
    std::vector<int> devices_;
    std::vector<AttocubeActor> actors_;

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
