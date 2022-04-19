//
// Created by george on 11/16/21.
//

//
// Created by george on 11/16/21.
//

#include <attocube_hardware_interface/attocube_actors.h>
#include <attocube_hardware_interface/attocube_device_manager.h>

int main( int argc, char ** argv ){
    rclcpp::init(argc, argv);

    // One command line argument which is the device number
    if(argc < 3){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"), "needs the index of the device to open and the axis number, exiting");
        return -1;
    }
    else if( argc > 3){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"), "too many arguments given, this just needs the device index and the axis number exiting");
        return -1;
    }
    int axis_id = std::stoi(argv[2]);
    if(axis_id > 2 || axis_id < 0) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"), "axis id can only be between 0 and 2. exiting");
        return -1;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"), "\n\nAxis Limit cycler starting\n\nThis will move a linear actor across the limits (+/- 15mm) at the highest frequency and amplitude\n\n\nTo start please manually move the actor to the zero (middle) position and press enter to continue...." );
    std::cin.ignore();

    AttocubeDeviceManager device_manager;
    int device_id = std::stoi(argv[1]);
    auto devs_available = device_manager.getDevicesAvailable();
    if(device_id > devs_available - 1){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"), "Device ID requested: " << device_id << " when only " << devs_available << " devices are available. Exiting");
        return -1;
    }
    device_manager.setupAllDevices();

    Bln32 on = 1, off = 0;  // constants
    Int32 amp   = 45000;    // Amplitude 45 V
    Int32 freq  = 5000000;  // Frequency 5 kHz
    int devHndl = device_id + 1;        // Device handle
    int rc      = NCB_Ok;   // status code

    auto actor = AttocubeActor(devHndl, axis_id, "id_joint", ECSx5050, amp, freq);
    if(actor.enableActor(true) && actor.resetActor()) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"),
                           "Using device: " << device_id << " starting motion across the whole workspace");
        actor.setDesiredPosition(0.015);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        actor.setDesiredPosition(-0.015);
        std::this_thread::sleep_for(std::chrono::seconds(10));
        actor.setDesiredPosition(0);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("attocube_actor_limits_cycle"), "Completed motion");
    }
}


