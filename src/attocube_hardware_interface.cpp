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
    }
}

void AttocubeHardwareInterface::getConfigFromParam() {

}

void AttocubeHardwareInterface::setupDevices() {

}

void AttocubeHardwareInterface::readPositions() {

}

void AttocubeHardwareInterface::writePositions() {

}

void AttocubeHardwareInterface::getActorFromName(std::string &joint_name, int &device, int &axis) {

}

void AttocubeHardwareInterface::getDevicesAvailable() {
    EccInfo * info = nullptr;
    unsigned int dev_count = ECC_Check( &info );

    ROS_INFO_STREAM(dev_count << " devices found\n");
    for ( unsigned int i = 0; i < dev_count; i++ ) {
        printf( "Device found: No=%d ", i );
        if ( info[i] .locked ) {
            ROS_ERROR_STREAM( "Device found: " << i << " [locked]\n" );
        }
        else {
            ROS_INFO_STREAM( "Device found: " << i << " ID: " << info[i].id );
            devices_.push_back(i);
        }
    }
    ECC_ReleaseInfo();

}


int main( int argc, char ** argv ) {
    return 1;
}