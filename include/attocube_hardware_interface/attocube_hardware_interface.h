//
// Created by george on 11/5/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <attocube_hardware_interface/attocube_actors.h>
#include <attocube_hardware_interface/attocube_device_manager.h>
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

class AttocubeHardwareInterface : public hardware_interface::RobotHW{
public:
    AttocubeHardwareInterface(ros::NodeHandle& nh);
    ~AttocubeHardwareInterface();

    // Hardware interface functions
    /** @brief sets up the control interface according to the urdf and yaml param file
     *
     */
    void register_interfaces();
    void read(ros::Duration duration);
    void write(ros::Duration duration);

    void debug_status();

    bool setupDevice();
    bool callbackSrvEnableActors(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool callbackSrvResetActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    bool callbackSrvHomeActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    bool callbackSrvStartROSControl(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

    AttocubeDeviceManager device_manager_;
    std::vector<AttocubeActor> actors_;
    ros::NodeHandle nh_;
    ros::ServiceServer service_enable_actors_;
    ros::ServiceServer service_reset_actors_;
    ros::ServiceServer service_home_actors_;
    ros::ServiceServer service_trigger_ros_control_;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    urdf::Model urdf_model_;

    bool enabled_ros_control = false;
    std::vector<double> current_position_, current_velocity_;
    std::vector<double> command_position_;
    std::vector<std::string> joint_names_;
    double effort_placeholder_ = 0;


};

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
