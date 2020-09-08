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

static std::string getECCErrorMessage( int code )
{
    switch( code ) {
        case NCB_Ok:                   return "";
        case NCB_Error:                return "Unspecified error";
        case NCB_Timeout:              return "Communication timeout";
        case NCB_NotConnected:         return "No active connection to device";
        case NCB_DriverError:          return "Error in comunication with driver";
        case NCB_DeviceLocked:         return "Device is already in use by other";
        case NCB_InvalidParam:         return "Parameter out of range";
        case NCB_FeatureNotAvailable:  return "Feature not available";
        default:                       return "Unknown error code";
    }
}

class AttocubeHardwareInterface {
public:
    AttocubeHardwareInterface(ros::NodeHandle& nh);
    ~AttocubeHardwareInterface();
    void getConfigFromParam(); // Setup the number of controller and the actor settings
    void getHardcodedConfig();
    void setupDevices();
    void setupActors();
    void printActorInformation(int& dev, int& axis);
    void readPositions();
    void writePositions();
    int getDevicesAvailable();
    void getActorFromName(std::string& joint_name, int& device, int& axis);

    ros::NodeHandle nh_;
    std::vector<int> devices_, devices_available_;
    std::vector<AttocubeActor> actors_;

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
