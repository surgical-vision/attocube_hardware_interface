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
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <angles/angles.h>
#include <std_srvs/SetBool.h>

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

int toNanoMetre(double metre){
    return (int) metre * 1e9;
}

double toMetre(int nano_metre){
    return (double) nano_metre / 1e9;
}

int toMicroDegree(double radian){
    return (int) (angles::to_degrees(radian) * 1e6);
}

double toRadian(int micro_degree){
    return (double) angles::from_degrees(micro_degree / 1e6);
}

class AttocubeHardwareInterface {
public:
    AttocubeHardwareInterface(ros::NodeHandle& nh);
    ~AttocubeHardwareInterface();
    void getConfigFromParam(); // Setup the number of controller and the actor settings
    void getHardcodedConfig();
    void setupDevices();
    void setupActors();
    void setupInterfaces();
    void printActorInformation(int& dev, int& axis);
    void readPositions();
    void writePositions();
    int getDevicesAvailable();
    bool enableActors(bool& on);

    // ROS side
    void generateJointStateMsg(sensor_msgs::JointState& msg);
    bool readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    bool callbackSrvEnableActors(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

    ros::NodeHandle nh_;
    std::vector<int> devices_, devices_available_;
    std::map<std::string, AttocubeActor> actors_;
    ros::Publisher publisher_joint_state_;
    ros::Subscriber subscriber_joint_trajectory_;
    ros::ServiceServer service_enable_actors_;

};


#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
