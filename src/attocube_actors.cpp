//
// Created by george on 10/19/20.
//

#include <attocube_hardware_interface/attocube_actors.h>

AttocubeActor::AttocubeActor(int device, int axis, std::string joint_name, int actor_type) :
        device_(device), axis_(axis), joint_name_(std::move(joint_name)), actor_type_(actor_type) {
    amplitude_ = 40333; // 40V
    frequency_ = 2000000; // 2khz

    setConfig();
};

AttocubeActor::AttocubeActor(int device, int axis, std::string joint_name, int actor_type, int voltage, int frequency) :
        device_(device), axis_(axis), joint_name_(std::move(joint_name)), actor_type_(actor_type), frequency_(frequency),
        amplitude_(voltage){
    setConfig();
}

bool AttocubeActor::setConfig() {
    return setActorType(actor_type_) && setActorAmplitude(amplitude_) && setActorFrequency(frequency_);
}

int *AttocubeActor::getType() {
    return &actor_type_;
}

double AttocubeActor::estimateVelocity() {
    getRawCurrentPosition();
    double velocity;
    if (actor_type_ == ECSx5050) {
        velocity = toMetre(current_position_ - previous_position_) / (current_read_time_ - previous_read_time_).seconds();
    } else {
        velocity =
                toRadian(current_position_ - previous_position_) / (current_read_time_ - previous_read_time_).seconds();
    }
    return velocity;
}

AttocubeActor::~AttocubeActor() {
    enableActor(false);

}

bool AttocubeActor::setRawDesiredPosition(int desired_position) {
    desired_position_ = desired_position;
    int rc;
    rc = ECC_controlTargetPosition(device_, axis_, &desired_position_, 1);
    if (rc != NCB_Ok) {
        return false;
    }
    return true;
}

bool AttocubeActor::setDesiredPosition(double desired_position) {
    if (actor_type_ == ECSx5050) {
        return setRawDesiredPosition(toNanoMetre(desired_position));
    } else {
        return setRawDesiredPosition(toMicroDegree(desired_position));
    }
}

bool AttocubeActor::setActorFrequency(int frequency) {
    frequency_ = frequency;
    int rc;
    rc = ECC_controlFrequency(device_, axis_, &frequency_, 1);
    if (rc != NCB_Ok) {
        return false;
    }
    return true;
}

bool AttocubeActor::setActorAmplitude(int amplitude) {
    amplitude_ = amplitude;
    int rc;
    rc = ECC_controlAmplitude(device_, axis_, &amplitude_, 1);
    if (rc != NCB_Ok) {
        return false;
    }
    return true;
}

bool AttocubeActor::enableActor(bool on) {
    int val = (int) on;
    int rc = 0;

    // Set the current position as the desired position;
    setRawDesiredPosition(getRawCurrentPosition());

    rc = ECC_controlOutput(device_, axis_, &val, 1);
    if (rc != NCB_Ok) {
        return false;
    }
    rc = ECC_controlMove(device_, axis_, &val, 1);
    if (rc != NCB_Ok) {
        return false;
    }
    enabled_ = on;
    return true;
}

int AttocubeActor::getRawCurrentPosition() {
    int rc;

    rc = ECC_getPosition(device_, axis_, &current_position_);
    // Moves the current position to previous to get a new value;
    previous_position_ = current_position_;
    previous_read_time_ = current_read_time_;

    current_read_time_ = rclcpp::Clock().now();
    if (rc != NCB_Ok) {
        return NAN;
    }
    return current_position_;
}

double AttocubeActor::getCurrentPosition() {
    if (actor_type_ == ECSx5050) {
        return toMetre(getRawCurrentPosition());
    } else {
        return toRadian(getRawCurrentPosition());
    }
}

bool AttocubeActor::findEOTLimits(int timeout, bool full_speed=false) {
    int rc, on = 1, off = 0, eot_found = 0;
    rclcpp::Time time_start, time_now;
    rclcpp::Duration max_duration(timeout, 0);
    int original_amp, original_freq;

    if(full_speed){
        original_amp = amplitude_;
        original_freq = frequency_;
        setActorAmplitude(MAX_AMPLITUDE);
        setActorFrequency(MAX_FREQUENCY);
    }

    // Auto reset the zero position when known
    ECC_controlAutoReset(device_, axis_, &on, 1);
    // Auto update the ref position when known
    ECC_controlReferenceAutoUpdate(device_, axis_, &on, 1);
    // Set output to deactivate on finding the end of travel
    rc = ECC_controlEotOutputDeactive(device_, axis_, &on, 1);
    if (rc == NCB_Ok) {
        // set continuous movement in the desired direction
        if (home_direction_ == 0) {
            ECC_controlContinousBkwd(device_, axis_, &on, 1);
            ECC_getStatusEotBkwd(device_, axis_, &eot_found);
            time_start = rclcpp::Clock().now();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Start time for " << joint_name_ << " is " << time_start.seconds());
            while (eot_found != 1) {
                ECC_getStatusEotBkwd(device_, axis_, &eot_found);
                time_now = rclcpp::Clock().now();
                if ((time_now - time_start) > max_duration) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Finding EOT travel limit exceeded timeout\n");
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Start time for " << joint_name_ << " is " << time_start.seconds() << " and end time is " << time_now.seconds() <<
                    " with difference of " << (time_now - time_start).seconds());
                    break;
                }
            }
            if (eot_found == 1) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"EOT limit for " << joint_name_ << " has been found");
            }
            ECC_controlContinousBkwd(device_, axis_, &off, 1);
        } else if (home_direction_ == 1) {
            ECC_controlContinousFwd(device_, axis_, &on, 1);
            ECC_getStatusEotFwd(device_, axis_, &eot_found);
            time_start = rclcpp::Clock().now();
            while (eot_found != 1) {
                ECC_getStatusEotFwd(device_, axis_, &eot_found);
                time_now = rclcpp::Clock().now();
                if ((time_now - time_start) > max_duration) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Finding EOT travel limit exceeded timeout\n");
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"), "Start time for " << joint_name_ << " is " << time_start.seconds() << " and end time is " << time_now.seconds() <<
                                                                                                           " with difference of " << (time_now - time_start).seconds());
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            if (eot_found == 1) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"EOT limit for " << joint_name_ << " has been found");
            }
            ECC_controlContinousFwd(device_, axis_, &off, 1);

        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),
                    "Direction was not 0 for backward or 1 for forward, set the direction to either of those values");
        }

        // Reenable output now the EOT has been found
        if (enableActor(true)) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actor for " << joint_name_ << " has been re-enabled");
        }

    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actor for " << joint_name_
                                      << " joint failed to set reaching the EOT to deactivate the output with the error message: "
                                      << getECCErrorMessage(rc));
    }
    if(full_speed){
        setActorAmplitude(original_amp);
        setActorFrequency(original_freq);
    }
    return eot_found;
}

bool AttocubeActor::setActorType(int type) {
    int rc;
    rc = ECC_controlActorSelection(device_, axis_,
                                   &type, 1);
    if(rc != NCB_Ok){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Failed to set actor type with following error message:\n\t" << getECCErrorMessage(rc));
        return false;
    }

    return true;
}

bool AttocubeActor::resetActor() {
    // Disables actor, resets the position, sets the desired to zero and reenables the actor
    int rc;
    bool was_enabled = checkEnabled();
    if(was_enabled) {
        enableActor(false);
    }
    rc = ECC_setReset(device_, axis_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    int current_position = getRawCurrentPosition();
    setRawDesiredPosition(current_position);
    if(was_enabled) {
        enableActor(true);
    }

    if(rc == NCB_Ok && abs(getRawCurrentPosition()) <= 50){ //Within 50 nanometre of 0 should maybe be parammed but should be fine for now.
        return true;
    } else{
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actor failed to reset\nCurrent raw position: " << getRawCurrentPosition() << "\nECC error message: " << getECCErrorMessage(rc));
        return false;
    }
}

bool AttocubeActor::checkEnabled() const {
    // TODO: change this to actually check with the controller if the actor is enabled
    return enabled_;
}

bool AttocubeActor::checkReference() {
    int status = 0, ref_position = 0, rc;

    rc = ECC_getStatusReference(device_, axis_, &status);
    if(rc != NCB_Ok){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actor for " << joint_name_ << " joint failed to get status reference with the error message: " << getECCErrorMessage(rc));
    }
    ECC_getReferencePosition(device_, axis_, &ref_position);
    reference_valid_ = status;
    reference_position_ = ref_position;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Reference for Joint: " << joint_name_ << "\nStatus: " << status << "\nPosition: " << ref_position);

    return reference_valid_;
}

bool AttocubeActor::findRefPosition(int timeout) {
    int rc, on = 1, off = 0, ref_found = 0;
    rclcpp::Time start_time;
    rclcpp::Duration max_duration(timeout, 0);
    bool timeout_reached = false;


    // set continuous movement in the desired direction
    if (home_direction_ == 0) {
        ECC_controlContinousBkwd(device_, axis_, &on, 1);
        ECC_getStatusReference(device_, axis_, &ref_found);
        start_time = rclcpp::Clock().now();
        while (ref_found != 1) {
            ECC_getStatusReference(device_, axis_, &ref_found);
            if ((rclcpp::Clock().now() - start_time) > max_duration) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Finding EOT travel limit exceeded timeout");
                timeout_reached = true;
                break;
            }
        }
        if (ref_found == 1) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"EOT limit for " << joint_name_ << " has been found");
        }
        ECC_controlContinousBkwd(device_, axis_, &off, 1);
    } else if (home_direction_ == 1) {
        ECC_controlContinousFwd(device_, axis_, &on, 1);
        ECC_getStatusReference(device_, axis_, &ref_found);
        start_time = rclcpp::Clock().now();
        while (ref_found != 1) {
            ECC_getStatusReference(device_, axis_, &ref_found);
            if ((rclcpp::Clock().now() - start_time) > max_duration) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Finding ref position exceeded timeout");
                timeout_reached = true;
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(5));
        }
        if (ref_found == 1) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Ref position for " << joint_name_ << " has been found");
        }
        ECC_controlContinousFwd(device_, axis_, &off, 1);

    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),
                "Direction was not 0 for backward or 1 for forward, set the direction to either of those values");
    }

    // Reenable output now the EOT has been found
    if (checkReference() && enableActor(true)) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AttocubeHardwareInterface"),"Actor for " << joint_name_ << " has been re-enabled");
        return ref_found;
    } else {
        if (timeout_reached) {
            return false;
        }
    }
    return ref_found;
}


// Util functions
std::string getECCErrorMessage( int code )
{
    switch( code ) {
        case NCB_Ok:                   return "";
        case NCB_Error:                return "Unspecified error";
        case NCB_Timeout:              return "Communication timeout";
        case NCB_NotConnected:         return "No active connection to device";
        case NCB_DriverError:          return "Error in comunication with driver";
        case NCB_DeviceLocked:         return "Device is already in use by other";
        case NCB_InvalidParam:         return "Parameter out of range";
        case NCB_FeatureNotAvailable:  return "Feature not available";
        default:                       return "Unknown error code";
    }
}

int toNanoMetre(double metre){
    return (int) (metre * 1e9);
}

double toMetre(int nano_metre){
    return (double) (nano_metre * 1e-9);
}

int toMicroDegree(double radian){
    return (int) (angles::to_degrees(radian) * 1e6);
}

double toRadian(int micro_degree){
    return (double) angles::from_degrees(micro_degree * 1e-6);
}