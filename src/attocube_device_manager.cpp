//
// Created by george on 10/19/20.
//

#include "attocube_hardware_interface/attocube_device_manager.h"

AttocubeDeviceManager::AttocubeDeviceManager() {

}

AttocubeDeviceManager::~AttocubeDeviceManager() {
    if(!devices_.empty()){
        for(const auto &dev : devices_){
            RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Closing device number: " << dev);
            ECC_Close(dev);
        }
    } else{
        RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"No devices were initialised");
    }

}

int AttocubeDeviceManager::getDevicesAvailable() {
    EccInfo * info = nullptr;
    unsigned int dev_count = ECC_Check( &info );

    for ( unsigned int i = 0; i < dev_count; i++ ) {
        if ( info[i] .locked ) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Device found: " << i << " [locked]\n" );
        }
        else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Device found: " << i << " ID: " << info[i].id );
            devices_available_.push_back(info[i].id);
        }
    }
    if(dev_count != devices_available_.size()){
        RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Some devices are locked and will not be initialised");
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Found " << dev_count << " devices");
    ECC_ReleaseInfo();
    return devices_available_.size();
}

bool AttocubeDeviceManager::setupAllDevices() {
    int rc; // Return code from ECC call
    int handle; // device handle
    for(int i = 0; i < devices_available_.size(); i++){
        rc = ECC_Connect(i, &handle);
        if(rc == NCB_Ok){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Device " << i << " connected\n\tID: " << devices_available_[i] << "\n\tHandle: " << handle);
            devices_.emplace_back(handle);
        }
    }
    return devices_.size() == devices_available_.size();
}

bool AttocubeDeviceManager::checkDevicesInitialised() {
    return !devices_.empty();
}
