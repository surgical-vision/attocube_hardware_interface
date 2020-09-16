//
// Created by george on 9/11/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <attocube_hardware_interface/attocube_hardware_interface.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

class AttocubeRosInterface{
public:
    AttocubeRosInterface(ros::NodeHandle& nh);
    bool hardcodeSetupDevice();
    void generateJointStateMsg(sensor_msgs::JointState& msg);
    bool readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr &msg, int point_index);
    void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    bool callbackSrvEnableActors(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool callbackSrvResetActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    bool callbackSrvHomeActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    void publishJointState();
    void callbackExecuteFollowJointTrajectory(control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
    bool callbackSrvEnableActionTrajectory(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    AttocubeHardwareInterface interface_;
    ros::NodeHandle nh_;
    ros::Publisher publisher_joint_state_;
    ros::Subscriber subscriber_joint_trajectory_;
    ros::ServiceServer service_enable_actors_;
    ros::ServiceServer service_reset_actors_;
    ros::ServiceServer service_home_actors_;
    ros::ServiceServer service_enable_action_trajectory_;

    // Action server
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_follow_joint_trajectory_;
    control_msgs::FollowJointTrajectoryFeedback feedback_follow_joint_;
    control_msgs::FollowJointTrajectoryResult result_follow_joint_trajectory_;

};
#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H
