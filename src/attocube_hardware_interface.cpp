//
// Created by george on 9/3/20.
//

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

    }

}

void AttocubeHardwareInterface::writePositions() {

}

void AttocubeHardwareInterface::getActorFromName(std::string &joint_name, int &device, int &axis) {

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
        case int(ECC_actorLinear):   type_name = "Linear";
        case int(ECC_actorGonio):    type_name = "Goniometer";
        case int(ECC_actorRot):      type_name = "Rotation";
        default:                     type_name = "N/A";
    }
    ROS_INFO_STREAM("For Device " << dev << " and axis " << axis << "\n\tName: " << name << "\n\tType: " << type_name << "\n\tType ID: " << type);
    free(name);
}

void AttocubeHardwareInterface::getHardcodedConfig() {
    ROS_WARN_STREAM("Using hardcoded configuration");
    actors_.emplace_back(0, 0, "x_axis", 6);
    actors_.emplace_back(0, 1, "y_axis", 6);
    actors_.emplace_back(0, 2, "goni", 10);
}

void AttocubeHardwareInterface::setupActors() {
    ROS_INFO_STREAM("Setting up actors from config");
    if(!actors_.empty()){
        for(auto& actor : actors_){
            ECC_controlActorSelection(devices_[actor.device_], actor.axis_,
                                      actor.getType(), 1);
        }
    } else{
        ROS_ERROR_STREAM("No actors are configured");
    }

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
        ros::Duration(0.1).sleep();
        auto axis = 0;
        interface.printActorInformation(interface.devices_[0], axis);
        axis = 1;
        interface.printActorInformation(interface.devices_[0], axis);
        axis = 2;
        interface.printActorInformation(interface.devices_[0], axis);
        ros::Duration(0.1).sleep();
    } else{
        ROS_ERROR_STREAM("No devices available");
    }

    return 0;
}