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
#include <controller_manager/controller_manager.h>
#include <control_toolbox/filters.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <std_msgs/Duration.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>

class AttocubeRosInterface : public hardware_interface::RobotHW{
public:
    AttocubeRosInterface(ros::NodeHandle& nh);
    ~AttocubeRosInterface();

    // Hardware interface functions
    void register_interfaces();
    void read(ros::Duration duration);
    void write(ros::Duration duration);

    bool hardcodeSetupDevice();
    void generateJointStateMsg(sensor_msgs::JointState& msg);
    bool readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr &msg, int point_index);
    void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    bool callbackSrvEnableActors(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool callbackSrvResetActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    bool callbackSrvHomeActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    void publishJointState();

    AttocubeHardwareInterface interface_;
    ros::NodeHandle nh_;
    ros::Publisher publisher_joint_state_;
    ros::Subscriber subscriber_joint_trajectory_;
    ros::ServiceServer service_enable_actors_;
    ros::ServiceServer service_reset_actors_;
    ros::ServiceServer service_home_actors_;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    std::vector<double> current_position_, current_velocity_;
    std::vector<double> command_position_;
    std::vector<std::string> joint_names_;
    double effort_placeholder_ = 0;


};
#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ROS_INTERFACE_H
