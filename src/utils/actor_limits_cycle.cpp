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
    Int32 amp   = 45000;    // Amplitude 45 V
    Int32 freq  = 5000000;  // Frequency 5 kHz
    int devHndl = 0;        // Device handle
    int rc      = NCB_Ok;   // status code

    int devNo = selectDevice();
    if ( devNo < 0 ) {
        printf( "No devices found\n" );
        return 1;
    }

    rc = ECC_Connect( device_id, &devHndl );
    rc = ECC_controlAmplitude( device_id, 0, &amp, on );
    rc = ECC_controlFrequency( device_id, 0, &freq, on );
    rc = ECC_setReset( device_id, 0 );
    rc = ECC_controlOutput( device_id, 0, &on, on );

    // Forward for a second then backwards
    rc = ECC_controlContinousFwd( device_id, 0, &on, on );
    rclcpp::sleep_for(1e9);
    rc = ECC_controlContinousFwd( device_id, 0, &off, on );
    rc = ECC_controlContinousBkwd( device_id, 0, &on, on );
    rclcpp::sleep_for(1e9);
    rc = ECC_controlContinousBkwd( device_id, 0, &off, on );
    rc = ECC_Close( device_id)

}

