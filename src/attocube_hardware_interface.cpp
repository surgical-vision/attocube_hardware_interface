//
// Created by george on 11/5/20.
//

#include "attocube_hardware_interface/attocube_hardware_interface.h"

AttocubeHardwareInterface::AttocubeHardwareInterface(ros::NodeHandle& nh) {
    nh_ = nh;
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeHardwareInterface::callbackSrvEnableActors, this);
    service_reset_actors_ = nh_.advertiseService("reset_actors", &AttocubeHardwareInterface::callbackSrvResetActors, this);
    service_home_actors_ = nh_.advertiseService("home_actors", &AttocubeHardwareInterface::callbackSrvHomeActors, this);
    service_trigger_ros_control_ = nh_.advertiseService("enable_ros_control", &AttocubeHardwareInterface::callbackSrvStartROSControl, this);
}

AttocubeHardwareInterface::~AttocubeHardwareInterface() {
    ROS_ERROR_STREAM("Interface shutting down");
    //TODO: stop and disable all actors
    //TODO: close connection to controllers
}

bool AttocubeHardwareInterface::setupDevice() {
    device_manager_.getDevicesAvailable();
    return device_manager_.setupAllDevices();
}

void AttocubeHardwareInterface::register_interfaces() {
    /* Load urdf and configs from param.
     * Intilise the actors from the list of joints
    */

    // TODO: Check they are empty and send out a warning they will be overwritten
    actors_.clear();
    joint_names_.clear();

    XmlRpc::XmlRpcValue joints;
    int axis, device, type;
    std::string joint_name;

    if ( nh_.getParam("joints", joints) ) {
        for(int i = 0; i < joints.size(); i++) {
            XmlRpc::XmlRpcValue sublist = joints[i];
            axis = sublist["axis"];
            device = sublist["device"]; //TODO: check if device exists eg. devices_available_.size < device;
            type = sublist["type"];
            joint_name = (std::string)sublist["name"];
            ROS_INFO_STREAM("Initialising Actor with the following config: \n\tDevice: " << device
                << "\n\tAxis: " << axis << "\n\tJoint name: " << joint_name << "\n\tType: " << type); //TODO: Change type to string
            actors_.emplace_back(device, axis, joint_name, type);
            joint_names_.emplace_back(joint_name);
        }
        ROS_INFO_STREAM("Actors initialised");
    } else {
        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
        throw std::runtime_error("No joint name specification");
    }

    // Initialise interfaces from urdf with limits
    if (!(urdf_model_.initParam("robot_description"))) {
        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
        throw std::runtime_error("No URDF model available");
    }

    urdf::JointConstSharedPtr current_joint;
    int i = 0;
    current_position_.assign(actors_.size(), 0);
    current_velocity_.assign(actors_.size(), 0);
    command_position_.assign(actors_.size(), 0);
    for(auto & j_name : joint_names_){
        // For each actor add the state variables to the state handle and desired to the position interface
        hardware_interface::JointStateHandle state_handle(j_name,
                                                          &(current_position_[i]),
                                                          &(current_velocity_[i]),
                                                          &(effort_placeholder_));
        jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
                jnt_state_interface.getHandle(j_name), &command_position_[i]);

        jnt_pos_interface.registerHandle(position_joint_handle);

        // TODO: Add joint limits from URDF

        i++;
    }
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_state_interface);
    ROS_INFO_STREAM("Interfaces Registered but not enabled");
}


void AttocubeHardwareInterface::read(ros::Duration duration) {
    /*
     * Read from each actor
     * convert to ROS units
     * add to relevant vector
    */
    int i = 0;
    for(auto &actor : actors_){
        current_position_[i] = actor.getCurrentPosition();
        current_velocity_[i] = actor.estimateVelocity();
        i++;
        // This calls get raw position again, a way to speed up will be to remove the call or check if the previous call was recent enough it doesn't require an update
    }

}

void AttocubeHardwareInterface::write(ros::Duration duration) {
    /*
     * Read relevant vector
     * Check within limits (when implemented)
     * send to actors
     */
    int i = 0;
    for(auto &actor : actors_){
        //check the values with a joint limits interface
        actor.setDesiredPosition(command_position_[i]);
        i++;
    }
}

bool AttocubeHardwareInterface::callbackSrvEnableActors(std_srvs::SetBool::Request &request,
                                                        std_srvs::SetBool::Response &response) {
    std::stringstream message;
    if(device_manager_.checkDevicesInitialised()) {
        bool enable = request.data;
        bool success = false;
        for (auto &actor : actors_) {
            success = actor.enableActor(enable);
            if(!success){
                if(enable) {
                    message << "Actor for " << actor.joint_name_ << " failed to enable\n";
                } else{
                    message << "Actor for " << actor.joint_name_ << " failed to disable\n";
                }
            }
        }
        if (message.gcount() == 0){
            // No messages means all were a success (hopefully)
            if(enable) {
                message << "All actors successfully enabled\n";
            } else{
                message << "All actors successfully disabled\n";
            }
            response.message = message.str();
            response.success = true;
            return true;
        } else{
            response.message = message.str();
            response.success = false;
            return false;
        }
    } else{
        ROS_ERROR_STREAM("Devices have not been initialised, can't enable actors without the controllers");
        return false;
    }
}

bool AttocubeHardwareInterface::callbackSrvResetActors(std_srvs::Trigger::Request &request,
                                                       std_srvs::Trigger::Response &response) {
    ROS_WARN_STREAM("Reseting position for each actor, current position will be set to zero and the reference value will be invalid");
    std::stringstream message;
    if(device_manager_.checkDevicesInitialised()) {
        bool success = false;
        for (auto &actor : actors_) {
            success = actor.resetActor();
            if(!success){
                message << "Actor for " << actor.joint_name_ << " failed to reset\n";
            }
        }
        if (message.tellg() == 0){
            // No messages means all were a success (hopefully)
            message << "All actors successfully reset\n";
            response.message = message.str();
            response.success = true;
            return true;
        } else{
            response.message = message.str();
            response.success = false;
            return false;
        }
    } else{
        ROS_ERROR_STREAM("Devices have not been initialised, can't enable actors without the controllers");
        return false;
    }
}

bool AttocubeHardwareInterface::callbackSrvHomeActors(std_srvs::Trigger::Request &request,
                                                      std_srvs::Trigger::Response &response) {
    ROS_WARN_STREAM("Homing the position for each actor, ensure each actor is free from potential collisions in their home direction");
    std::stringstream message;
    if(device_manager_.checkDevicesInitialised()) {
        bool success = false;
        for (auto &actor : actors_) {
            success = actor.findEOTLimits(10); //TODO: param or send with the request
            if(!success){
                message << "Actor for " << actor.joint_name_ << " failed to find limit\n";
            }
        }
        if (message.tellg() == 0){
            // No messages means all were a success (hopefully)
            ROS_WARN("Rezeroing all actors");
            for (auto &actor : actors_) {
                actor.setRawDesiredPosition(0);
            }
            message << "All actors successfully reset\n";
            response.message = message.str();
            response.success = true;
            return true;
        } else{
            response.message = message.str();
            response.success = false;
            return false;
        }
    } else{
        ROS_ERROR_STREAM("Devices have not been initialised, can't home actors without the controllers");
        return false;
    }
}

bool AttocubeHardwareInterface::callbackSrvStartROSControl(std_srvs::SetBool::Request &request,
                                                           std_srvs::SetBool::Response &response) {
    if(request.data){ // If request to enable
        // Check the actors have been configured, enabled and homed before enabling the ros control interface
        bool ref, enabled;
        int ready = 0;
        for(auto &actor : actors_){
            ref = actor.checkReference();
            enabled = actor.checkEnabled();
            if (ref && enabled){
                ready++;
            }
        }
        if(ready == actors_.size()){
            ROS_WARN_STREAM("All actors ready, control interface will be enabled");
            enabled_ros_control = true;
            response.success = true;
            response.message = "ROS control interface enabled";
            return true;
        } else{
            response.success = false;
            response.message = "Actors not ready, failed to intialise ros control interface";
            return false;
        }
    } else{
        // request to disable ros control
        enabled_ros_control = false;
        // Setting each actor to stop but not disengage
        for(auto &actor : actors_){
            actor.setRawDesiredPosition(actor.getRawCurrentPosition());
        }
        response.message = "ROS control interface disabled, actors set to halt";
        response.success = true;
        return true;
    }

}

void AttocubeHardwareInterface::debug_status() {
    std::stringstream status;
    for(int i = 0; i < joint_names_.size(); i++){
        status << "Joint: " << joint_names_[i] << "\tCurrent Position: " << current_position_[i] << "Desired position: " << command_position_[i] << std::endl;
    }
    ROS_DEBUG_STREAM(status.str());
}

int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh("/xy_stage");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    AttocubeHardwareInterface interface(nh);
    bool success = interface.setupDevice();
    if(!success){
        ROS_ERROR_STREAM("Failed to setup controllers, check they are not locked\n\tExiting");
        spinner.stop();
        return -1;
    }
    interface.register_interfaces();

    controller_manager::ControllerManager cm(&interface, nh);
    ros::Time current_time, previous_time = ros::Time::now();
    ros::Duration elapsed_time;

    ros::Rate rate(100); //param this

    while(ros::ok()){
        current_time = ros::Time::now();
        elapsed_time = ros::Duration(current_time - previous_time);
        previous_time = current_time;

        if(interface.enabled_ros_control){
            interface.read(elapsed_time);
            cm.update(current_time, elapsed_time);
            interface.write(elapsed_time);
        } else{
            ROS_DEBUG_STREAM_THROTTLE(10, "control interface not running"); // add status of actors and devices
        }
        interface.debug_status();
        ROS_DEBUG_STREAM_THROTTLE(1, "Rate cycle time: " << rate.cycleTime().toSec());
        rate.sleep();
    }

    spinner.stop();
    return 0;
}