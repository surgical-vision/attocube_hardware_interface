//
// Created by george on 11/16/21.
//

#include <attocube_hardware_interface/attocube_actors.h>
#include <attocube_hardware_interface/attocube_device_manager.h>

int main( int argc, char ** argv ){
    rclcpp::init(argc, argv);

    // One command line argument which is the device number
    if(argc < 2){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_device_identifier"), "needs the index of the device to open, exiting");
        return -1;
    }
    else if( argc > 2){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_device_identifier"), "too many arguments given, this just needs the device index, exiting");
        return -1;
    }

    AttocubeDeviceManager device_manager;
    std::vector<AttocubeActor> actors;

    int device_id = std::stoi(argv[1]);
    auto devs_available = device_manager.getDevicesAvailable();
    if(device_id > devs_available - 1){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_device_identifier"), "Device ID requested: " << device_id << " when only " << devs_available << " devices are available. Exiting");
        return -1;
    }
    device_manager.setupAllDevices();

    Bln32 on = 1, off = 0;  // constants
    Int32 amp   = 30333;    // Amplitude 30.333 V
    Int32 freq  = 2222222;  // Frequency 2.22 kHz
    int devHndl = 0;        // Device handle
    int rc      = NCB_Ok;   // status code

    auto actor = AttocubeActor(device_id, 0, "id_joint", ECSx5050, amp, freq);
    actor.enableActor(true);
    actor.resetActor();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("attocube_device_identifier"), "Using device: " << device_id << " starting motion for 1 second in either direction");
    actor.setDesiredPosition(0.002);
    std::this_thread::sleep_for(std::chrono::seconds (3));
    actor.setDesiredPosition(-0.002);
    std::this_thread::sleep_for(std::chrono::seconds (6));
    actor.setDesiredPosition(0);
    std::this_thread::sleep_for(std::chrono::seconds (3));
    rc = ECC_controlContinousBkwd( device_id, 0, &off, on );
    RCLCPP_INFO_STREAM(rclcpp::get_logger("attocube_device_identifier"), "Completed motion");

}