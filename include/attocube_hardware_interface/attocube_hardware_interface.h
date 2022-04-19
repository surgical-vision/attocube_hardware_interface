//
// Created by george on 11/5/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H

#include <builtin_interfaces/msg/duration.h>
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.h>
#include <angles/angles.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

// System
#include <memory>
#include <vector>

// ros2_control hardware_interface
#include "hardware_interface/actuator.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

#include <attocube_hardware_interface/attocube_actors.h>
#include <attocube_hardware_interface/attocube_device_manager.h>
// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>

class AttocubeHardwareInterface : public hardware_interface::SystemInterface{
public:

    RCLCPP_SHARED_PTR_DEFINITIONS(AttocubeHardwareInterface)

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& system_info) final;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::status get_status() const final
    {
        return status_;
    }

    std::string get_name() const final
    {
        return info_.name;
    }

    hardware_interface::return_type start() final;
    hardware_interface::return_type stop() final;
    hardware_interface::return_type read() final;
    hardware_interface::return_type write() final;

//    // ROS1
//    AttocubeHardwareInterface(ros::NodeHandle& nh);
//    ~AttocubeHardwareInterface();
//
//    // Hardware interface functions
//    /** @brief sets up the control interface according to the urdf and yaml param file
//     *
//     */
//    void register_interfaces();
//    void read(ros::Duration duration);
//    void write(ros::Duration duration);
//
//    void debug_status();
//
    bool setupDevice();
//    bool callbackSrvEnableActors(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
//    bool callbackSrvResetActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
//    bool callbackSrvHomeActors(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
//    bool callbackSrvStartROSControl(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

private:
    hardware_interface::HardwareInfo info_;
    hardware_interface::status status_;
    rclcpp::Clock clock_;
    AttocubeDeviceManager device_manager_;
    std::vector<AttocubeActor> actors_;

    uint32_t runtime_state_;
    bool controllers_initialized_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_enable_actors_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_reset_actors_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_home_actors_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_ros_control_;

    bool enabled_ros_control = false;
    std::vector<double> current_position_, current_velocity_;
    std::vector<double> command_position_;
    std::vector<std::string> joint_names_;
};

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(AttocubeHardwareInterface, hardware_interface::SystemInterface)

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_HARDWARE_INTERFACE_H
