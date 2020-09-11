//
// Created by george on 9/11/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <attocube_hardware_interface/attocube_hardware_interface.h>

class AttocubeRosInterface{
public:
    AttocubeRosInterface(ros::NodeHandle& nh);
    void generateJointStateMsg(sensor_msgs::JointState& msg);
    bool readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    bool callbackSrvEnableActors(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

    AttocubeHardwareInterface interface_;
    ros::NodeHandle nh_;
    ros::Publisher publisher_joint_state_;
    ros::Subscriber subscriber_joint_trajectory_;
    ros::ServiceServer service_enable_actors_;
};
#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H
