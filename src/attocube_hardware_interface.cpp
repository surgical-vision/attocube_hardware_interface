//
// Created by george on 9/3/20.
//

#include <attocube_hardware_interface/attocube_hardware_interface.h>

AttocubeHardwareInterface::AttocubeHardwareInterface(ros::NodeHandle& nh) {
    nh_ = nh;
}

AttocubeHardwareInterface::~AttocubeHardwareInterface() {
    if(enableActors(false)){
        ROS_WARN_STREAM("All actors disabled");
    } else{
        ROS_WARN_STREAM("Actors were not disabled");
    }
    if(!devices_.empty()){
        for(const auto &dev : devices_){
            ROS_WARN_STREAM("Closing device number: " << dev);
            ECC_Close(dev);
        }
    } else{
        ROS_WARN_STREAM("No devices were initialised");
    }
}

void AttocubeHardwareInterface::getConfigFromParam() {
    XmlRpc::XmlRpcValue actors;
    nh_.getParam("/attocube_hardware_interface/actors", actors);
    ROS_INFO_STREAM(actors << "\n\n");
    ROS_INFO_STREAM("Number of actors: " << actors.size());
//    for(int i = 0; i < actors.size(); i++){
//        auto actor_param = actors[i];
//        for(int j = 0; j < actor_param.size(); j++){
//            ROS_INFO_STREAM(actor_param[j]);
//        }
//        int axis = actor_param["axis"];
//        std::string name = actor_param["joint_name"];
//        ROS_INFO_STREAM("Axis: " << axis << "  Joint Name: " << name);
//    }

}

void AttocubeHardwareInterface::setupDevices() {
    int rc; // Return code from ECC call
    int handle; // device handle
    for(int i = 0; i < devices_available_.size(); i++){
        rc = ECC_Connect(i, &handle);
        if(rc == NCB_Ok){
            ROS_INFO_STREAM("Device " << i << " connected\n\tID: " << devices_available_[i] << "\n\tHandle: " << handle);
            devices_.emplace_back(handle);
        }
    }
}

void AttocubeHardwareInterface::readPositions() {
    int rc;
    for(auto& actor : actors_){
        actor.second.resetPostion();
        rc = ECC_getPosition(actor.second.device_, actor.second.axis_, &actor.second.current_position_);
        actor.second.current_read_time_ = ros::Time::now();
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to enable output with the error message: " << getECCErrorMessage(rc));
        }
    }
}

void AttocubeHardwareInterface::readSinglePosition(std::string &joint_name) {
    int rc;
    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        actor->second.resetPostion();
        rc = ECC_getPosition(actor->second.device_, actor->second.axis_, &actor->second.current_position_);
        actor->second.current_read_time_ = ros::Time::now();
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor->first << " joint failed to enable output with the error message: " << getECCErrorMessage(rc));
        }
    }
}

void AttocubeHardwareInterface::writePositions() {
    int rc;
    for(auto& actor : actors_){
        rc = ECC_controlTargetPosition(actor.second.device_, actor.second.axis_, &actor.second.desired_position_, 1);
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to set the desired position with the error message: " << getECCErrorMessage(rc));
        }
    }
}

void AttocubeHardwareInterface::writeSinglePosition(std::string &joint_name) {
    int rc;
    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        rc = ECC_controlTargetPosition(actor->second.device_, actor->second.axis_, &actor->second.desired_position_, 1);
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor->first << " joint failed to set the desired position with the error message: " << getECCErrorMessage(rc));
        }
    }
}

int AttocubeHardwareInterface::getDevicesAvailable() {
    EccInfo * info = nullptr;
    unsigned int dev_count = ECC_Check( &info );

    ROS_INFO_STREAM(dev_count << " devices found\n");
    for ( unsigned int i = 0; i < dev_count; i++ ) {
        if ( info[i] .locked ) {
            ROS_ERROR_STREAM( "Device found: " << i << " [locked]\n" );
        }
        else {
            ROS_INFO_STREAM( "Device found: " << i << " ID: " << info[i].id );
            devices_available_.push_back(info[i].id);
        }
    }
    ECC_ReleaseInfo();
    return devices_available_.size();
}

void AttocubeHardwareInterface::printActorInformation(int &dev, int &axis) {
    // Iterate over connected devices
    char* name;
    name = (char*)malloc(20);
    ECC_actorType type;
    int rc;

    rc = ECC_getActorName(dev, axis, name);
    if(rc != NCB_Ok){
        auto error = getECCErrorMessage(rc);
        ROS_ERROR_STREAM(error);
    }
    ECC_getActorType(dev, axis, &type);
    if(rc != NCB_Ok){
        auto error = getECCErrorMessage(rc);
        ROS_ERROR_STREAM(error);
    }

    std::string type_name;
    switch (type) {
        case int(ECC_actorLinear):   type_name = "Linear"; break;
        case int(ECC_actorGonio):    type_name = "Goniometer"; break;
        case int(ECC_actorRot):      type_name = "Rotation"; break;
        default:                     type_name = "N/A";
    }
    ROS_INFO_STREAM("For Device " << dev << " and axis " << axis << "\n\tName: " << name << "\n\tType: " << type_name << "\n\tType ID: " << type);
    free(name);
}

void AttocubeHardwareInterface::getHardcodedConfig() {
    ROS_WARN_STREAM("Using hardcoded configuration");
    actors_.emplace("x_axis", AttocubeActor(1, 0, "x_axis", ECSx5050));
    actors_.emplace("y_axis", AttocubeActor(1, 1, "y_axis", ECSx5050));
    actors_.emplace("z_axis", AttocubeActor(2, 0, "z_axis", ECSx5050));
    actors_.emplace("goni", AttocubeActor(1, 2, "goni", ECGp5050));
}

void AttocubeHardwareInterface::setupActors() {
    ROS_INFO_STREAM("Setting up actors from config");
    int rc;
    if(!actors_.empty()){
        for(auto& actor : actors_){
            rc = ECC_controlActorSelection(actor.second.device_, actor.second.axis_,
                                      actor.second.getType(), 1);
            if(rc != NCB_Ok){
                ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to set the control amplitude with the error message: " << getECCErrorMessage(rc));
            }
        }
    } else{
        ROS_ERROR_STREAM("No actors are configured");
    }
    pushActorSettings();
}

bool AttocubeHardwareInterface::enableActors(bool on) {
    readPositions();
    for(auto& actor : actors_) {
        actor.second.desired_position_ = actor.second.current_position_;
    }

    writePositions();

    int val = (int)on;
    int rc = 0, total_rc = 0;
    for(auto& actor : actors_){
        rc = ECC_controlOutput(actor.second.device_, actor.second.axis_, &val, 1);
        total_rc += rc;
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to enable output with the error message: " << getECCErrorMessage(rc));
        }
        rc = ECC_controlMove(actor.second.device_, actor.second.axis_, &val, 1);
        total_rc += rc;
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to setup control move with the error message: " << getECCErrorMessage(rc));
        }
    }
    if(total_rc == 0){
        return true;
    } else{
        ROS_ERROR_STREAM("Some or all actuators have not been enabled");
        return false;
    }
}

bool AttocubeHardwareInterface::enableSingleActor(std::string &joint_name, bool on) {
    readPositions();
    int val = (int)on;
    int rc = 0, total_rc = 0;
    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        actor->second.desired_position_ = actor->second.current_position_;
        writeSinglePosition(actor->second.joint_name_);
        rc = ECC_controlOutput(actor->second.device_, actor->second.axis_, &val, 1);
        total_rc += rc;
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor->first << " joint failed to enable output with the error message: " << getECCErrorMessage(rc));
        }
        rc = ECC_controlMove(actor->second.device_, actor->second.axis_, &val, 1);
        total_rc += rc;
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor->first << " joint failed to setup control move with the error message: " << getECCErrorMessage(rc));
        }
    }
    if(total_rc == 0){
        return true;
    } else{
        ROS_ERROR_STREAM("Some or all actuators have not been enabled");
        return false;
    }
}

void AttocubeHardwareInterface::getReferenceValues() {
    int status = 0, ref_position = 0, rc;
    for(auto& actor : actors_){
        rc = ECC_getStatusReference(actor.second.device_, actor.second.axis_, &status);
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to get status reference with the error message: " << getECCErrorMessage(rc));
        }
        ECC_getReferencePosition(actor.second.device_, actor.second.axis_, &ref_position);
        actor.second.reference_valid_ = status;
        actor.second.refernce_position_ = ref_position;
        ROS_INFO_STREAM("Reference for Joint: " << actor.first << "\nStatus: " << status << "\nPosition: " << ref_position);
    }
}

void AttocubeHardwareInterface::pushActorSettings() {
    int rc;
    for(auto& actor : actors_){
        rc = ECC_controlAmplitude(actor.second.device_, actor.second.axis_, &actor.second.amplitude_, 1);
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to set the control amplitude with the error message: " << getECCErrorMessage(rc));
        }
        rc = ECC_controlFrequency(actor.second.device_, actor.second.axis_, &actor.second.frequency_, 1);
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to set the control frequency with the error message: " << getECCErrorMessage(rc));
        }
        rc = ECC_controlTargetRange(actor.second.device_, actor.second.axis_, &actor.second.target_range, 1);
        if(rc != NCB_Ok){
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to set the target range with the error message: " << getECCErrorMessage(rc));
        }
    }
}

bool AttocubeHardwareInterface::findEOTLimits(std::string &joint_name, int direction, int timeout) {
    int rc, on = 1, off = 0, eot_found = 0;
    ros::Time start_time;
    ros::Duration max_duration(timeout);

    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        // Set output to deactivate on finding the end of travel
        rc = ECC_controlEotOutputDeactive(actor->second.device_, actor->second.axis_, &on, 1);
        if(rc == NCB_Ok) {
            // set continous movement in the desired direction
            if (direction == 0){
                ECC_controlContinousBkwd(actor->second.device_, actor->second.axis_, &on, 1);
                ECC_getStatusEotBkwd(actor->second.device_, actor->second.axis_, &eot_found);
                start_time = ros::Time::now();
                while (eot_found != 1){
                    ECC_getStatusEotBkwd(actor->second.device_, actor->second.axis_, &eot_found);
                    if((ros::Time::now() - start_time) > max_duration){
                        ROS_WARN_STREAM("Finding EOT travel limit exceeded timeout");
                        break;
                    }
                }
                if(eot_found == 1){
                    ROS_INFO_STREAM("EOT limit for " << actor->first << " has been found");
                }
                ECC_controlContinousBkwd(actor->second.device_, actor->second.axis_, &off, 1);
            } else if (direction == 1){
                ECC_controlContinousFwd(actor->second.device_, actor->second.axis_, &on, 1);
                ECC_getStatusEotFwd(actor->second.device_, actor->second.axis_, &eot_found);
                start_time = ros::Time::now();
                while (eot_found != 1){
                    ECC_getStatusEotFwd(actor->second.device_, actor->second.axis_, &eot_found);
                    if((ros::Time::now() - start_time) > max_duration){
                        ROS_WARN_STREAM("Finding EOT travel limit exceeded timeout");
                        break;
                    }
                    ros::Duration(0.05).sleep();
                }
                if(eot_found == 1){
                    ROS_INFO_STREAM("EOT limit for " << actor->first << " has been found");
                }
                ECC_controlContinousFwd(actor->second.device_, actor->second.axis_, &off, 1);

            } else{
                ROS_ERROR_STREAM("Direction was not 0 for backward or 1 for forward, set the direction to either of those values");
            }

            // Reenable output now the EOT has been found
            if(enableSingleActor(actor->second.joint_name_, true)){
                ROS_INFO_STREAM("Actor for " << actor->first << " has been re-enabled");
            }

        } else{
            ROS_ERROR_STREAM("Actor for " << actor->first << " joint failed to set reaching the EOT to deactivate the output with the error message: " << getECCErrorMessage(rc));
        }
    }
    return eot_found;
}

bool AttocubeHardwareInterface::homeAllActors() {
    bool success = false;
    for(auto& actor : actors_){
        ROS_INFO_STREAM("Finding limit for " << actor.first);
        success = findEOTLimits(actor.second.joint_name_, actor.second.home_direction_, 10);
        if(!success){
            ROS_WARN_STREAM("Unable to find limit for " << actor.first);
        }
        // Set desired position to zero after finding the limit
        actor.second.desired_position_ = 0;
    }

    ROS_INFO_STREAM("Found each eot now returning to 0");
    writePositions();
    waitForAllActorsHalt(20);
    return success;
}

bool AttocubeHardwareInterface::waitForAllActorsHalt(int timeout) {
    int ind_moving = -1, total_moving = -1;
    ros::Time start_time = ros::Time::now();
    ros::Duration max_duration(timeout);
    ros::Rate rate(2);

    while(total_moving != actors_.size()){
        total_moving = 0;
        readPositions();
        for(auto& actor : actors_){
//            ECC_getStatusMoving(actor.second.device_, actor.second.axis_, &ind_moving);
            ECC_getStatusTargetRange(actor.second.device_, actor.second.axis_, &ind_moving);

            total_moving += ind_moving;
            ROS_DEBUG_STREAM("Current moving status of " << actor.first << ": " << ind_moving <<
                "\nCurrent position: " << actor.second.current_position_ << "\tDesired position: " << actor.second.desired_position_ << std::endl);
        }
        readPositions();
        if((ros::Time::now() - start_time) > max_duration){
            ROS_WARN_STREAM("Waiting to complete move exceeded timeout");
            break;
        }
        rate.sleep();
    }
    return total_moving == actors_.size();
}

bool AttocubeHardwareInterface::sendDesiredPosition(std::string& joint_name, double value) {
    // Assume the position is in metres or radians
    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        if(actor->second.actor_type_ == ECC_actorLinear) {
            actor->second.desired_position_ = toNanoMetre(value);
        } else{
            actor->second.desired_position_ = toMicroDegree(value);
        }
        writePositions();
        return true;
    } else {
        ROS_ERROR_STREAM("Unknown actor for " << joint_name);
        return false;
    }
}

bool AttocubeHardwareInterface::checkMoving(std::string &joint_name) {
    int ind_moving = -1;
    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        ECC_getStatusMoving(actor->second.device_, actor->second.axis_, &ind_moving);
    } else {
        ROS_ERROR_STREAM("Unknown actor for " << joint_name);
    }
    return ind_moving == 1;
}

double AttocubeHardwareInterface::getCurrentPosition(std::string &joint_name) {
    readSinglePosition(joint_name);
    auto actor = actors_.find(joint_name);
    if(actor != actors_.end()){
        if(actor->second.actor_type_ == ECC_actorLinear) {
            return toMetre(actor->second.current_position_);
        } else{
            return toRadian(actor->second.current_position_);
        }
    } else {
        ROS_ERROR_STREAM("Unknown actor for " << joint_name);
    }
    return 0;
}

bool AttocubeHardwareInterface::resetPositions() {
    ROS_WARN_STREAM("Reseting position for each actor, current position will be set to zero and the reference value will be invalid");
    int ind_rc, total_rc = 0;
    for(auto& actor : actors_){
        ind_rc = ECC_setReset(actor.second.device_, actor.second.axis_);
        total_rc += ind_rc;
    }
    return total_rc == NCB_Ok;
}

bool AttocubeHardwareInterface::allActorsEnabled() {
    int rc = 0, total_enabled = 0, is_enabled = -1;
    for(auto& actor : actors_) {
        rc = ECC_controlOutput(actor.second.device_, actor.second.axis_, &is_enabled, 0);
        if (rc != NCB_Ok) {
            ROS_ERROR_STREAM("Actor for " << actor.first << " joint failed to check output with the error message: "
                                          << getECCErrorMessage(rc));
        }
        total_enabled += is_enabled;
    }
    return total_enabled == actors_.size();
}

bool AttocubeHardwareInterface::allActorsReferenced() {
    getReferenceValues();
    int is_ref = -1, total_ref = 0;
    for(auto& actor : actors_) {
        is_ref = (int) actor.second.reference_valid_;
        total_ref += is_ref;
    }
    return total_ref == actors_.size();
}
