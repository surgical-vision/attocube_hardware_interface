//
// Created by george on 11/5/20.
//

#include "attocube_hardware_interface/attocube_hardware_interface.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

bool AttocubeHardwareInterface::setupDevice() {
    int available = device_manager_.getDevicesAvailable();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Number of devices available: " << available);
    if (available == 0){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"No devices available, exiting");
        return false;
    }
    return device_manager_.setupAllDevices();
}

hardware_interface::return_type
AttocubeHardwareInterface::configure(const hardware_interface::HardwareInfo &system_info) {
    clock_ = rclcpp::Clock(RCL_STEADY_TIME);
    info_ = system_info;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Entering configure");
    controllers_initialized_ = false;
    
    if(setupDevice()){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Attocube controllers initialised");
    } else{
        RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"),
                     "Unable to initialise the Attocube controllers");
        return hardware_interface::return_type::ERROR;
    }
    ;
    
    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"),
                         "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"),
                         "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"), "Joint '%s' has %d state interface. 2 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"),
                         "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"),
                         "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),
                           "Initialising Actor with the following config: \n\tDevice: " << joint.parameters.find("device")->second
                            << "\n\tAxis: " << joint.parameters.find("axis")->second << "\n\tJoint name: " << joint.name << "\n\tType: " << joint.parameters.find("type")->second
                            << "\n\tVoltage: " << joint.parameters.find("voltage")->second << "\n\tFrequency: " << joint.parameters.find("frequency")->second);

        int device, axis, type, voltage, frequency;
        device = std::stoi(joint.parameters.find("device")->second);
        type = std::stoi(joint.parameters.find("type")->second);
        axis = std::stoi(joint.parameters.find("axis")->second);
        voltage = std::stoi(joint.parameters.find("voltage")->second);
        frequency = std::stoi(joint.parameters.find("frequency")->second);

        actors_.emplace_back(device, axis, joint.name, type, voltage, frequency);
        joint_names_.emplace_back(joint.name);

    }
    // initialize
    current_position_.assign(actors_.size(), 0);
    current_velocity_.assign(actors_.size(), 0);
    command_position_.assign(actors_.size(), 0);

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> AttocubeHardwareInterface::export_state_interfaces() {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Setting up state interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Setting joint: " << info_.joints[i].name << " at index " << i);
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &current_position_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &current_velocity_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AttocubeHardwareInterface::export_command_interfaces() {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Setting up command interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Setting joint: " << info_.joints[i].name << " at index " << i);
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type AttocubeHardwareInterface::start() {
    RCLCPP_INFO(rclcpp::get_logger("AttocubeHardwareInterface"), "Starting ...please wait...");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    if(!device_manager_.checkDevicesInitialised()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Attocube controllers are not initialised");
        status_ = hardware_interface::status::UNKNOWN;
        return hardware_interface::return_type::ERROR;
    }

    // Enable actors
    bool enable = true;
    bool success = false;
    for (auto &actor : actors_) {
        success = actor.enableActor(enable);
        if(!success){
            RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),
                               "Actor for " << actor.joint_name_ << " failed to enable" << std::endl);
            status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        } else{
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Actor for " << actor.joint_name_ << " enabled");
        }
    }

    // Home actors
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Starting home procedure for actors");
    success = false;
    for (auto &actor : actors_) {
        if(actor.actor_type_ != ECR5050){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Starting home procedure for linear actor: " << actor.joint_name_);
            success = actor.findEOTLimits(20); //TODO: param or send with the request
            if(!success){
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),  "Actor for " << actor.joint_name_ << " failed to find limit\n");
                status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
        }
        else if(actor.actor_type_ == ECR5050){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Starting home procedure for rotational actor: " << actor.joint_name_);
            success = actor.findRefPosition(20); //TODO: param or send with the request
            if(!success){
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),  "Actor for " << actor.joint_name_ << " failed to find reference position\n");
                status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
        }
        else{
            RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Unknown actor type on device " << actor.device_ << " and axis " << actor.axis_);
            status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
    }

    status_ = hardware_interface::status::STARTED;
    for (auto &actor : actors_) {
        actor.setRawDesiredPosition(0);
    }

    std::copy(current_position_.begin(), current_position_.end(), command_position_.begin());
    RCLCPP_INFO(rclcpp::get_logger("AttocubeHardwareInterface"), "System successfully started!");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AttocubeHardwareInterface::stop() {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Attocube hardware interface will be stopped, actors will be disabled now" << std::endl);
    bool enable = false;
    bool success = false;
    for (auto &actor : actors_) {
        success = actor.enableActor(enable);
        if(!success){
            if(enable) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Actor for " << actor.joint_name_ << " failed to enable" << std::endl);
            } else{
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Actor for " << actor.joint_name_ << " failed to disable" << std::endl);
            }
            status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AttocubeHardwareInterface::read() {
    int i = 0;
    for(auto &actor : actors_){
        current_position_[i] = actor.getCurrentPosition();
        current_velocity_[i] = actor.estimateVelocity();
        i++;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AttocubeHardwareInterface::write() {
    int i = 0;
    for(auto &actor : actors_){
        //check the values with a joint limits interface
        actor.setDesiredPosition(command_position_[i]);
        i++;
    }
    return hardware_interface::return_type::OK;
}
