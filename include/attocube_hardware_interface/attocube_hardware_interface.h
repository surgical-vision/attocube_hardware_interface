//
// Created by george on 9/3/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H

#include <string>
#include <vector>
#include <attocube_hardware_interface/attocube_actors.h>
#include <attocube_hardware_interface/attocube_utils.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>
#include <ecc.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>

class AttocubeHardwareInterface {
public:
    AttocubeHardwareInterface(ros::NodeHandle& nh);
    ~AttocubeHardwareInterface();
    // Setup functions
    void getConfigFromParam(); // Setup the number of controller and the actor settings
    void getHardcodedConfig();
    void setupDevices();
    void setupActors();

    // Controller communication functions
    void readPositions();
    void readSinglePosition(std::string& joint_name);
    void writePositions();
    void writeSinglePosition(std::string& joint_name);
    int getDevicesAvailable();
    bool enableActors(bool on);
    bool enableSingleActor(std::string& joint_name, bool on);
    void getReferenceValues();
    void pushActorSettings();

    // Helper Functions
    void printActorInformation(int& dev, int& axis);
    bool findEOTLimits(std::string &joint_name, int direction, int timeout);
    bool homeAllActors();
    bool waitForAllActorsHalt(int timeout);
    bool sendDesiredPosition(std::string& joint_name, double value);
    bool checkMoving(std::string& joint_name);
    double getCurrentPosition(std::string& joint_name);
    bool resetPositions();

    ros::NodeHandle nh_;
    std::vector<int> devices_, devices_available_;
    std::map<std::string, AttocubeActor> actors_;

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
