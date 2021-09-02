//
// Created by george on 11/5/20.
//

#include "attocube_hardware_interface/attocube_hardware_interface.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

//AttocubeHardwareInterface::AttocubeHardwareInterface(ros::NodeHandle& nh) {
//    nh_ = nh;
//    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeHardwareInterface::callbackSrvEnableActors, this);
//    service_reset_actors_ = nh_.advertiseService("reset_actors", &AttocubeHardwareInterface::callbackSrvResetActors, this);
//    service_home_actors_ = nh_.advertiseService("home_actors", &AttocubeHardwareInterface::callbackSrvHomeActors, this);
//    service_trigger_ros_control_ = nh_.advertiseService("enable_ros_control", &AttocubeHardwareInterface::callbackSrvStartROSControl, this);
//}
//
//AttocubeHardwareInterface::~AttocubeHardwareInterface() {
//    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Interface shutting down");
//    for(auto &actor : actors_){
//        actor.setRawDesiredPosition(actor.getRawCurrentPosition());
//        actor.enableActor(false);
//    }
//}
//
bool AttocubeHardwareInterface::setupDevice() {
    int available = device_manager_.getDevicesAvailable();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Number of devices available: " << available);
    if (available == 0){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"No devices available, exiting");
        return false;
    }
    return device_manager_.setupAllDevices();
}
//
//void AttocubeHardwareInterface::register_interfaces() {
//    /* Load urdf and configs from param.
//     * Intilise the actors from the list of joints
//    */
//
//    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Registering joint interfaces");
//    // TODO: Check they are empty and send out a warning they will be overwritten
//    actors_.clear();
//    joint_names_.clear();
//
//    XmlRpc::XmlRpcValue joints;
//    int axis, device, type, voltage, frequency;
//    std::string joint_name;
//
//    if ( nh_.getParam("joints", joints) ) {
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"got actor settings from joint param");
//        for(int i = 0; i < joints.size(); i++) {
//            XmlRpc::XmlRpcValue sublist = joints[i];
//            axis = sublist["axis"];
//            device = sublist["device"]; //TODO: check if device exists eg. devices_available_.size < device;
//            type = sublist["type"];
//            voltage = sublist["voltage"];
//            frequency = sublist["frequency"];
//            joint_name = (std::string)sublist["name"];
//            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Initialising Actor with the following config: \n\tDevice: " << device
//                << "\n\tAxis: " << axis << "\n\tJoint name: " << joint_name << "\n\tType: " << type
//                << "\n\tVoltage: " << voltage << "\n\tFrequency: " << frequency); //TODO: Change the type field to string
//            actors_.emplace_back(device, axis, joint_name, type, voltage, frequency);
//            joint_names_.emplace_back(joint_name);
//        }
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actors initialised");
//    } else {
//        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
//        throw std::runtime_error("No joint name specification");
//    }
//
//    // Initialise interfaces from urdf with limits
//    if (!(urdf_model_.initParam("robot_description"))) {
//        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
//        throw std::runtime_error("No URDF model available");
//    }
//    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"URDF initialised");
//
//    urdf::JointConstSharedPtr current_joint;
//    int i = 0;
//    current_position_.assign(actors_.size(), 0);
//    current_velocity_.assign(actors_.size(), 0);
//    command_position_.assign(actors_.size(), 0);
//    for(auto & j_name : joint_names_){
//        // For each actor add the state variables to the state handle and desired to the position interface
//        hardware_interface::JointStateHandle state_handle(j_name,
//                                                          &(current_position_[i]),
//                                                          &(current_velocity_[i]),
//                                                          &(effort_placeholder_));
//        jnt_state_interface.registerHandle(state_handle);
//
//        hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
//                jnt_state_interface.getHandle(j_name), &command_position_[i]);
//
//        jnt_pos_interface.registerHandle(position_joint_handle);
//
//        // TODO: Add joint limits from URDF
//
//        i++;
//    }
//    registerInterface(&jnt_pos_interface);
//    registerInterface(&jnt_state_interface);
//    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Interfaces Registered but not enabled");
//}
//
//
//void AttocubeHardwareInterface::read(rclcpp::Duration duration) {
//    /*
//     * Read from each actor
//     * convert to ROS units
//     * add to relevant vector
//    */
//    int i = 0;
//    for(auto &actor : actors_){
//        current_position_[i] = actor.getCurrentPosition();
//        current_velocity_[i] = actor.estimateVelocity();
//        i++;
//        // This calls get raw position again, a way to speed up will be to remove the call or check if the previous call was recent enough it doesn't require an update
//    }
//
//}
//
//void AttocubeHardwareInterface::write(rclcpp::Duration duration) {
//    /*
//     * Read relevant vector
//     * Check within limits (when implemented)
//     * send to actors
//     */
//    int i = 0;
//    for(auto &actor : actors_){
//        //check the values with a joint limits interface
//        actor.setDesiredPosition(command_position_[i]);
//        i++;
//    }
//}
//
//bool AttocubeHardwareInterface::callbackSrvEnableActors(std_srvs::SetBool::Request &request,
//                                                        std_srvs::SetBool::Response &response) {
//    std::stringstream message;
//    if(device_manager_.checkDevicesInitialised()) {
//        bool enable = request.data;
//        bool success = false;
//        for (auto &actor : actors_) {
//            success = actor.enableActor(enable);
//            if(!success){
//                if(enable) {
//                    message << "Actor for " << actor.joint_name_ << " failed to enable" << std::endl;
//                } else{
//                    message << "Actor for " << actor.joint_name_ << " failed to disable" << std::endl;
//                }
//            }
//        }
//        if (message.str().empty()){
//            // No messages means all were a success (hopefully)
//            if(enable) {
//                message << "All actors successfully enabled";
//            } else{
//                message << "All actors successfully disabled";
//            }
//            response.message = message.str();
//            response.success = true;
//            return true;
//        } else{
//            response.message = message.str();
//            response.success = false;
//            return false;
//        }
//    } else{
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Devices have not been initialised, can't enable actors without the controllers");
//        return false;
//    }
//}
//
//bool AttocubeHardwareInterface::callbackSrvResetActors(std_srvs::Trigger::Request &request,
//                                                       std_srvs::Trigger::Response &response) {
//    RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Reseting position for each actor, current position will be set to zero and the reference value will be invalid");
//    std::stringstream message;
//    if(device_manager_.checkDevicesInitialised()) {
//        bool success = false;
//        for (auto &actor : actors_) {
//            success = actor.resetActor();
//            if(!success){
//                message << "Actor for " << actor.joint_name_ << " failed to reset\n";
//            }
//        }
//        if (message.str().empty()){
//            // No messages means all were a success (hopefully)
//            message << "All actors successfully reset\n";
//            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),message.str());
//            response.message = message.str();
//            response.success = true;
//            return true;
//        } else{
//            RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),message.str());
//            response.message = message.str();
//            response.success = false;
//            return false;
//        }
//    } else{
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Devices have not been initialised, can't reset actors without a connection to the controllers");
//        return false;
//    }
//}
//
//bool AttocubeHardwareInterface::callbackSrvHomeActors(std_srvs::Trigger::Request &request,
//                                                      std_srvs::Trigger::Response &response) {
//    RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Homing the position for each actor, ensure each actor is enabled and free from potential collisions in their home direction");
//    bool enabled;
//    int ready = 0;
//    for(auto &actor : actors_){
//        enabled = actor.checkEnabled();
//        if (enabled){
//            ready++;
//        }
//    }
//    std::stringstream message;
//    if(device_manager_.checkDevicesInitialised() && ready == actors_.size()) {
//        bool success = false;
//        for (auto &actor : actors_) {
//            if(actor.actor_type_ != ECR5050){
//                success = actor.findEOTLimits(20); //TODO: param or send with the request
//                if(!success){
//                    message << "Actor for " << actor.joint_name_ << " failed to find limit\n";
//                }
//            }
//            else if(actor.actor_type_ == ECR5050){
//                success = actor.findRefPosition(20); //TODO: param or send with the request
//                if(!success){
//                    message << "Actor for " << actor.joint_name_ << " failed to find reference position\n";
//                }
//            }
//            else{
//                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Unknown actor type for");
//            }
//        }
//        if (message.str().empty()){
//            // No messages means all were a success (hopefully)
//            ROS_WARN("Rezeroing all actors");
//            for (auto &actor : actors_) {
//                actor.setRawDesiredPosition(0);
//            }
//            message << "All actors successfully homed\n";
//            response.message = message.str();
//            response.success = true;
//            return true;
//        } else{
//            response.message = message.str();
//            response.success = false;
//            return true;
//        }
//    } else{
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Devices have not been initialised, can't home actors without the controllers");
//        return false;
//    }
//}
//
//bool AttocubeHardwareInterface::callbackSrvStartROSControl(std_srvs::SetBool::Request &request,
//                                                           std_srvs::SetBool::Response &response) {
//    bool check_force;
//    nh_.param<bool>("force_enable_control", check_force, false);
//
//    if (check_force){
//        RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Force enable control has been set to true, the actors will be seen as ready if just enabled");
//    }
//
//    if(request.data){ // If request to enable
//        // Check the actors have been configured, enabled and homed before enabling the ros control interface
//        bool ref, enabled;
//        int ready = 0;
//        for(auto &actor : actors_){
//            ref = actor.checkReference();
//            enabled = actor.checkEnabled();
//            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Joint: " << actor.joint_name_ << "\tReferenced: " << ref << "\tEnabled: " << enabled);
//            // If referenced and enabled or forced and enabled then they are ready
//            if ((ref && enabled) || (check_force && enabled)){
//                ready++;
//            }
//        }
//        if(ready == actors_.size()){
//            RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"All actors ready, control interface will be enabled");
//            enabled_ros_control = true;
//            response.success = true;
//            response.message = "ROS control interface enabled";
//            return true;
//        } else{
//            RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actors not ready, failed to initialise ros control interface");
//            response.success = false;
//            response.message = "Actors not ready, failed to initialise ros control interface";
//            return true; // It failed but doesn't return the message to say it failed if false
//        }
//    } else{
//        // request to disable ros control
//        enabled_ros_control = false;
//        // Setting each actor to stop but not disengage
//        for(auto &actor : actors_){
//            actor.setRawDesiredPosition(actor.getRawCurrentPosition());
//        }
//        response.message = "ROS control interface disabled, actors set to halt";
//        response.success = true;
//        return true;
//    }
//}
//
//void AttocubeHardwareInterface::debug_status() {
//    std::stringstream status;
//    for(int i = 0; i < joint_names_.size(); i++){
//        status << std::endl << "Joint: " << joint_names_[i] << "\tCurrent Position: " << current_position_[i] << "\tDesired position: " << command_position_[i];
//    }
//    status << std::endl;
//    ROS_DEBUG_STREAM_THROTTLE(1, status.str());
//}

hardware_interface::return_type
AttocubeHardwareInterface::configure(const hardware_interface::HardwareInfo &system_info) {
    clock_ = rclcpp::Clock(RCL_STEADY_TIME);
    info_ = system_info;
    
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

        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AttocubeHardwareInterface"), "Joint '%s' has %d state interface. 3 expected.",
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
        int device, axis, type, voltage, frequency;
        device = std::stoi(joint.parameters.find("device")->second);
        type = std::stoi(joint.parameters.find("type")->second);
        axis = std::stoi(joint.parameters.find("axis")->second);
        voltage = std::stoi(joint.parameters.find("voltage")->second);
        frequency = std::stoi(joint.parameters.find("frequency")->second);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),
                           "Initialising Actor with the following config: \n\tDevice: " << device
                            << "\n\tAxis: " << axis << "\n\tJoint name: " << joint.name << "\n\tType: " << type
                            << "\n\tVoltage: " << voltage << "\n\tFrequency: " << frequency); //TODO: Change the type field to string

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
            if(enable) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Actor for " << actor.joint_name_ << " failed to enable" << std::endl);
            } else{
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Actor for " << actor.joint_name_ << " failed to disable" << std::endl);
            }
            status_ = hardware_interface::status::UNKNOWN;
            return hardware_interface::return_type::ERROR;
        }
    }

    // Home actors
    success = false;
    for (auto &actor : actors_) {
        if(actor.actor_type_ != ECR5050){
            success = actor.findEOTLimits(20); //TODO: param or send with the request
            if(!success){
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),  "Actor for " << actor.joint_name_ << " failed to find limit\n");
                status_ = hardware_interface::status::UNKNOWN;
                return hardware_interface::return_type::ERROR;
            }
        }
        else if(actor.actor_type_ == ECR5050){
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
    return hardware_interface::return_type::ERROR;
}

//int main( int argc, char ** argv ) {
//    ros::init(argc, argv, "attocube_hardware_interface");
//    ros::NodeHandle nh;
//
//    ros::AsyncSpinner spinner(2);
//    spinner.start();
//
//    AttocubeHardwareInterface interface(nh);
//    bool success = interface.setupDevice();
//    if(!success){
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Failed to setup controllers, check they are not locked\n\tExiting");
//        spinner.stop();
//        return -1;
//    }
//    interface.register_interfaces();
//
//    controller_manager::ControllerManager cm(&interface, nh);
//    rclcpp::Time current_time, previous_time = rclcpp::Clock().now();
//    rclcpp::Duration elapsed_time;
//
//    RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Entering main control loop");
//    while(ros::ok()){
//        current_time = rclcpp::Clock().now();
//        elapsed_time = rclcpp::Duration(current_time - previous_time);
//        previous_time = current_time;
//
//        interface.read(elapsed_time);
//        cm.update(current_time, elapsed_time);
//
//        if(interface.enabled_ros_control){
//            interface.write(elapsed_time);
//        } else{
//            ROS_DEBUG_STREAM_THROTTLE(10, "control interface not running"); // add status of actors and devices
//        }
//        interface.debug_status();
//    }
//    RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Hardware interface shutting down");
//
//    spinner.stop();
//    return 0;
//}