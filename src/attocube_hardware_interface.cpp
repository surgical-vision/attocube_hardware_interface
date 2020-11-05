//
// Created by george on 11/5/20.
//

#include "attocube_hardware_interface/attocube_hardware_interface.h"

AttocubeHardwareInterface::AttocubeHardwareInterface(ros::NodeHandle& nh) {
    nh_ = nh;
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeHardwareInterface::callbackSrvEnableActors, this);
    service_reset_actors_ = nh_.advertiseService("reset_actors", &AttocubeHardwareInterface::callbackSrvResetActors, this);
    service_home_actors_ = nh_.advertiseService("home_actors", &AttocubeHardwareInterface::callbackSrvHomeActors, this);
    service_trigger_ros_control_ = nh_.advertiseService("enable_ros_control", &AttocubeHardwareInterface::callbackSrvStartROSControl, this);
}

AttocubeHardwareInterface::~AttocubeHardwareInterface() {
    ROS_ERROR_STREAM("Interface shutting down");
    //TODO: stop and disable all actors
    //TODO: close connection to controllers
}

void AttocubeHardwareInterface::register_interfaces() {
    // Load urdf and configs from param.

    // intialise actors from urdf and param file, where joint name will be taken


    // Check the actors have been configured, enabled and homed before adding them as a controllable interface
    if(!interface_.actors_.empty() && interface_.allActorsEnabled() && interface_.allActorsReferenced()){
        int i = 0;
        current_position_.assign(interface_.actors_.size(), 0);
        current_velocity_.assign(interface_.actors_.size(), 0);
        command_position_.assign(interface_.actors_.size(), 0);
        for(auto &actor : interface_.actors_){
            // For each actor add the state variables to the state handle and desired to the position interface
            hardware_interface::JointStateHandle state_handle(actor.first,
                                                              &(current_position_[i]),
                                                              &(current_velocity_[i]),
                                                              &(effort_placeholder_));
            jnt_state_interface.registerHandle(state_handle);

            hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
                    jnt_state_interface.getHandle(actor.first), &command_position_[i]);

            jnt_pos_interface.registerHandle(position_joint_handle);

            // TODO: Add joint limits from URDF

            i++;
        }
        registerInterface(&jnt_pos_interface);
        registerInterface(&jnt_state_interface);
    }
}

void AttocubeHardwareInterface::read(ros::Duration duration) {

}

void AttocubeHardwareInterface::write(ros::Duration duration) {

}

bool AttocubeHardwareInterface::callbackSrvEnableActors(std_srvs::SetBool::Request &request,
                                                        std_srvs::SetBool::Response &response) {
    return false;
}

bool AttocubeHardwareInterface::callbackSrvResetActors(std_srvs::Trigger::Request &request,
                                                       std_srvs::Trigger::Response &response) {
    return false;
}

bool AttocubeHardwareInterface::callbackSrvHomeActors(std_srvs::Trigger::Request &request,
                                                      std_srvs::Trigger::Response &response) {
    return false;
}

bool AttocubeHardwareInterface::callbackSrvStartROSControl(std_srvs::SetBool::Request &request,
                                                           std_srvs::SetBool::Response &response) {
    return false;
}
