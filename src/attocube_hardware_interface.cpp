//
// Created by george on 9/3/20.
//

#include <ecc.h>
#include "../include/attocube_hardware_interface/attocube_hardware_interface.h"

AttocubeHardwareInterface::AttocubeHardwareInterface(ros::NodeHandle& nh) {
    nh_ = nh;
}

AttocubeHardwareInterface::~AttocubeHardwareInterface() {
    if(!devices_.empty()){
        for(const auto &dev : devices_){
            ROS_WARN_STREAM("Closing device number: " << dev);
            ECC_Close(dev);
        }
    } else{
        ROS_WARN_STREAM("No devices were initialised");
    }
}

void AttocubeHardwareInterface::getConfigFromParam() {
    XmlRpc::XmlRpcValue actors;
    nh_.getParam("/attocube_hardware_interface/actors", actors);
    ROS_INFO_STREAM(actors << "\n\n");
    ROS_INFO_STREAM("Number of actors: " << actors.size());
//    for(int i = 0; i < actors.size(); i++){
//        auto actor_param = actors[i];
//        for(int j = 0; j < actor_param.size(); j++){
//            ROS_INFO_STREAM(actor_param[j]);
//        }
//        int axis = actor_param["axis"];
//        std::string name = actor_param["joint_name"];
//        ROS_INFO_STREAM("Axis: " << axis << "  Joint Name: " << name);
//    }

}

void AttocubeHardwareInterface::setupDevices() {
    int rc; // Return code from ECC call
    int handle; // device handle
    for(int i = 0; i < devices_available_.size(); i++){
        rc = ECC_Connect(i, &handle);
        if(rc == NCB_Ok){
            ROS_INFO_STREAM("Device " << i << " connected\n\tID: " << devices_available_[i] << "\n\tHandle: " << handle);
            devices_.emplace_back(handle);
        }
    }

}

void AttocubeHardwareInterface::readPositions() {
    for(auto& actor : actors_){
        actor.second.reset_postion();
        ECC_getPosition(actor.second.device_, actor.second.axis_, &actor.second.current_position_);
        actor.second.current_read_time_ = ros::Time::now();
    }

}

void AttocubeHardwareInterface::writePositions() {
    for(auto& actor : actors_){
        ECC_controlTargetPosition(actor.second.device_, actor.second.axis_, &actor.second.desired_position_, 1);
    }
}

int AttocubeHardwareInterface::getDevicesAvailable() {
    EccInfo * info = nullptr;
    unsigned int dev_count = ECC_Check( &info );

    ROS_INFO_STREAM(dev_count << " devices found\n");
    for ( unsigned int i = 0; i < dev_count; i++ ) {
        if ( info[i] .locked ) {
            ROS_ERROR_STREAM( "Device found: " << i << " [locked]\n" );
        }
        else {
            ROS_INFO_STREAM( "Device found: " << i << " ID: " << info[i].id );
            devices_available_.push_back(info[i].id);
        }
    }
    ECC_ReleaseInfo();
    return devices_available_.size();
}

void AttocubeHardwareInterface::printActorInformation(int &dev, int &axis) {
    // Iterate over connected devices
    char* name;
    name = (char*)malloc(20);
    ECC_actorType type;
    int rc;

    rc = ECC_getActorName(dev, axis, name);
    if(rc != NCB_Ok){
        auto error = getECCErrorMessage(rc);
        ROS_ERROR_STREAM(error);
    }
    ECC_getActorType(dev, axis, &type);
    if(rc != NCB_Ok){
        auto error = getECCErrorMessage(rc);
        ROS_ERROR_STREAM(error);
    }

    std::string type_name;
    switch (type) {
        case int(ECC_actorLinear):   type_name = "Linear"; break;
        case int(ECC_actorGonio):    type_name = "Goniometer"; break;
        case int(ECC_actorRot):      type_name = "Rotation"; break;
        default:                     type_name = "N/A";
    }
    ROS_INFO_STREAM("For Device " << dev << " and axis " << axis << "\n\tName: " << name << "\n\tType: " << type_name << "\n\tType ID: " << type);
    free(name);
}

void AttocubeHardwareInterface::getHardcodedConfig() {
    ROS_WARN_STREAM("Using hardcoded configuration");
    actors_.emplace("x_axis", AttocubeActor(0, 0, "x_axis", ECSx5050));
    actors_.emplace("y_axis", AttocubeActor(0, 1, "y_axis", ECSx5050));
    actors_.emplace("goni", AttocubeActor(0, 2, "goni", ECGp5050));
}

void AttocubeHardwareInterface::setupActors() {
    ROS_INFO_STREAM("Setting up actors from config");
    if(!actors_.empty()){
        for(auto& actor : actors_){
            ECC_controlActorSelection(devices_[actor.second.device_], actor.second.axis_,
                                      actor.second.getType(), 1);
        }
    } else{
        ROS_ERROR_STREAM("No actors are configured");
    }

}

void AttocubeHardwareInterface::generateJointStateMsg(sensor_msgs::JointState& msg) {
    msg.name.clear();
    msg.effort.clear();
    msg.position.clear();
    msg.velocity.clear();
    // TODO: convert to m or rad
    for(auto& actor : actors_){
        msg.name.push_back(actor.first);
        if (actor.second.actor_type_ == ECC_actorLinear){
            msg.position.push_back(toMetre(actor.second.current_position_));
            msg.velocity.push_back((double)(toMetre(actor.second.current_position_ - actor.second.previous_position_)) / (actor.second.current_read_time_ - actor.second.previous_read_time_).toSec());
        } else{
            msg.position.push_back(toRadian(actor.second.current_position_));
            msg.velocity.push_back((double)(toRadian(actor.second.current_position_ - actor.second.previous_position_)) / (actor.second.current_read_time_ - actor.second.previous_read_time_).toSec());
        }
    }
}

bool AttocubeHardwareInterface::readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    //Check if there are multiple points
    if(msg->points.size() > 1){
        ROS_WARN_STREAM("Found multiple trajectory points\nCurrently only supports a single desired position for each joint\nOnly the first point will be used");
    }
    for(int i = 0; i < msg->joint_names.size(); i++){
        auto actor = actors_.find(msg->joint_names[i]);
        if(actor != actors_.end()){
            // Found the actor with the joint name;
            if(actor->second.actor_type_ == ECC_actorLinear) {
                actor->second.desired_position_ = toNanoMetre(msg->points[i].positions[0]);
            } else{
                actor->second.desired_position_ = toMicroDegree(msg->points[i].positions[0]);
            }
        }
    }
    return false;
}

void AttocubeHardwareInterface::callbackJointTrajectory(const
        trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    readJointTrajectoryMsg(msg);
    writePositions();
}

bool AttocubeHardwareInterface::callbackSrvEnableActors(std_srvs::SetBool::Request &request,
                                                        std_srvs::SetBool::Response &response) {
    bool on = request.data;
    response.success = enableActors(on);
    return response.success;
}

bool AttocubeHardwareInterface::enableActors(bool& on) {
    readPositions();
    int val = (int)on;
    int rc = 0, total_rc = 0;
    for(auto& actor : actors_){
        actor.second.desired_position_ = actor.second.current_position_;
        rc = rc + ECC_controlMove(actor.second.device_, actor.second.axis_, &val, 1);
    }
    if(rc == 0){
        return true;
    } else{
        ROS_ERROR_STREAM("Some or all actuators have not been enabled");
        return false;
    }

}

void AttocubeHardwareInterface::setupInterfaces() {
    publisher_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    subscriber_joint_trajectory_ = nh_.subscribe("command_trajectory", 10, &AttocubeHardwareInterface::callbackJointTrajectory, this);
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeHardwareInterface::callbackSrvEnableActors, this);

}


int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh;
    AttocubeHardwareInterface interface(nh);

    if (interface.getDevicesAvailable() > 0) {
        interface.setupDevices();
        ROS_INFO_STREAM("Devices setup");
        interface.getHardcodedConfig();
        interface.setupActors();
        interface.setupInterfaces();
        ros::Duration(0.1).sleep();
        auto axis = 0;
        interface.printActorInformation(interface.devices_[0], axis);
        axis = 1;
        interface.printActorInformation(interface.devices_[0], axis);
        axis = 2;
        interface.printActorInformation(interface.devices_[0], axis);
        ros::Duration(0.1).sleep();

        sensor_msgs::JointState joint_state;
        while(ros::ok()){
            interface.generateJointStateMsg(joint_state);
            interface.publisher_joint_state_.publish(joint_state);
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }

    } else{
        ROS_ERROR_STREAM("No devices available");
    }

    return 0;
}