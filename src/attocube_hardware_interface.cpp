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
    /* Load urdf and configs from param.
     * Intilise the actors from the list of joints
    */
    XmlRpc::XmlRpcValue joints;
    int axis, device, type;
    std::string joint_name;
    if ( nh_.getParam("joints", joints) ) {
        for(int i = 0; i < joints.size(); i++) {
            XmlRpc::XmlRpcValue sublist = joints[i];
            axis = sublist["axis"];
            device = sublist["device"]; //TODO: check if device exists eg. devices_available_.size < device;
            type = sublist["type"];
            joint_name = (std::string)sublist["name"];
            actors_.emplace_back(device, axis, joint_name, type);
            // TODO: add joint_names vector to allocation instances in the same order
        }
    } else {
        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
        throw std::runtime_error("No joint name specification");
    }

    if (!(urdf_model_.initParam("robot_description"))) {
        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
        throw std::runtime_error("No URDF model available");
    }

//    urdf::JointConstSharedPtr current_joint;
//    // For each joint
//    // intialise actors from urdf and param file, where joint name will be taken
//
//
//    // Check the actors have been configured, enabled and homed before adding them as a controllable interface
//    if(!interface_.actors_.empty() && interface_.allActorsEnabled() && interface_.allActorsReferenced()){
//        int i = 0;
//        current_position_.assign(interface_.actors_.size(), 0);
//        current_velocity_.assign(interface_.actors_.size(), 0);
//        command_position_.assign(interface_.actors_.size(), 0);
//        for(auto &actor : interface_.actors_){
//            // For each actor add the state variables to the state handle and desired to the position interface
//            hardware_interface::JointStateHandle state_handle(actor.first,
//                                                              &(current_position_[i]),
//                                                              &(current_velocity_[i]),
//                                                              &(effort_placeholder_));
//            jnt_state_interface.registerHandle(state_handle);
//
//            hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
//                    jnt_state_interface.getHandle(actor.first), &command_position_[i]);
//
//            jnt_pos_interface.registerHandle(position_joint_handle);
//
//            // TODO: Add joint limits from URDF
//
//            i++;
//        }
//        registerInterface(&jnt_pos_interface);
//        registerInterface(&jnt_state_interface);
//    }
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

int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh("/xy_stage");
    AttocubeHardwareInterface interface(nh);
    interface.register_interfaces();
}