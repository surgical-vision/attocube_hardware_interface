//
// Created by george on 9/15/20.
//

#include <attocube_hardware_interface/deprecated_attocube_hardware_interface.h>

int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh;
    AttocubeHardwareInterface interface(nh);

    if (interface.getDevicesAvailable() > 0) {
        interface.setupDevices();
        ROS_INFO_STREAM("Devices setup");
        interface.getHardcodedConfig();
        ROS_INFO_STREAM("Setting up actors");
        interface.setupActors();
        ROS_INFO_STREAM("Enabling Actors");
        interface.getReferenceValues();
        interface.enableActors(true);
        ros::Duration(0.1).sleep();
        //interface.resetPositions();
        interface.readPositions();
        interface.getReferenceValues();

        interface.homeAllActors();

        ros::Duration(2).sleep();
        interface.getReferenceValues();
        ros::Duration(1).sleep();

        double current_position, previous_position, desired_position;
        bool is_moving;
        std::string joint_x = "x_axis", joint_y = "y_axis", joint_z = "z_axis", joint_goni = "goni";

        current_position = interface.getCurrentPosition(joint_x);
        is_moving = interface.checkMoving(joint_x);
        ROS_INFO_STREAM("Current position of " << joint_x << ": " << current_position << " m\nJoint is moving: " << is_moving);
        desired_position = current_position + 0.01; // Move 10 mm
        interface.sendDesiredPosition(joint_x, desired_position);
        is_moving = interface.checkMoving(joint_x);
        ROS_INFO_STREAM("joint moving to desired postion: " << is_moving);
        interface.waitForAllActorsHalt(5);
        previous_position = current_position;
        current_position = interface.getCurrentPosition(joint_x);
        ROS_INFO_STREAM("New position is " << current_position << " which is " << current_position-previous_position << " from the original position");


        // Coordinate motion

//        current_position = interface.getCurrentPosition(joint_goni);
//        is_moving = interface.checkMoving(joint_goni);
//        ROS_INFO_STREAM("Current position of " << joint_goni << ": " << angles::to_degrees(current_position) << " deg\nJoint is moving: " << is_moving);
//        desired_position = current_position + angles::from_degrees(1); // Move 10 mm
//        interface.sendDesiredPosition(joint_goni, desired_position);
//        is_moving = interface.checkMoving(joint_goni);
//        ROS_INFO_STREAM("joint moving to desired postion: " << is_moving);
//        ros::Duration(5).sleep();
//        previous_position = current_position;
//        current_position = interface.getCurrentPosition(joint_goni);
//        ROS_INFO_STREAM("New position is " << angles::to_degrees(current_position) << " which is " << angles::to_degrees(current_position-previous_position) << " from the original position");

    } else{
        ROS_ERROR_STREAM("No devices available");
    }

    return 0;
}