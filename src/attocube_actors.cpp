//
// Created by george on 10/19/20.
//

#include <attocube_hardware_interface/attocube_actors.h>

AttocubeActor::AttocubeActor(int device, int axis, std::string joint_name, int actor_type) :
        device_(device), axis_(axis), joint_name_(std::move(joint_name)), actor_type_(actor_type) {
    amplitude_ = 40333;
    frequency_ = 2222222;

    setActorType(actor_type_);
};

int *AttocubeActor::getType() {
    return &actor_type_;
}


double AttocubeActor::estimateVelocity() {
    getRawCurrentPosition();
    double velocity;
    if (actor_type_ == ECC_actorLinear) {
        velocity = toMetre(current_position_ - previous_position_) / (current_read_time_ - previous_read_time_).toSec();
    } else {
        velocity =
                toRadian(current_position_ - previous_position_) / (current_read_time_ - previous_read_time_).toSec();
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
    if (actor_type_ == ECC_actorLinear) {
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
    return true;
}

int AttocubeActor::getRawCurrentPosition() {
    int rc;

    rc = ECC_getPosition(device_, axis_, &current_position_);
    // Moves the current position to previous to get a new value;
    previous_position_ = current_position_;
    previous_read_time_ = current_read_time_;

    current_read_time_ = ros::Time::now();
    if (rc != NCB_Ok) {
        return NAN;
    }
    return current_position_;
}

double AttocubeActor::getCurrentPosition() {
    if (actor_type_ == ECC_actorLinear) {
        return toMetre(getRawCurrentPosition());
    } else {
        return toRadian(getRawCurrentPosition());
    }
    return 0;
}

bool AttocubeActor::findEOTLimits(int timeout) {
    int rc, on = 1, off = 0, eot_found = 0;
    ros::Time start_time;
    ros::Duration max_duration(timeout);

    // Set output to deactivate on finding the end of travel
    rc = ECC_controlEotOutputDeactive(device_, axis_, &on, 1);
    if (rc == NCB_Ok) {
        // set continous movement in the desired direction
        if (home_direction_ == 0) {
            ECC_controlContinousBkwd(device_, axis_, &on, 1);
            ECC_getStatusEotBkwd(device_, axis_, &eot_found);
            start_time = ros::Time::now();
            while (eot_found != 1) {
                ECC_getStatusEotBkwd(device_, axis_, &eot_found);
                if ((ros::Time::now() - start_time) > max_duration) {
                    ROS_WARN_STREAM("Finding EOT travel limit exceeded timeout");
                    break;
                }
            }
            if (eot_found == 1) {
                ROS_INFO_STREAM("EOT limit for " << joint_name_ << " has been found");
            }
            ECC_controlContinousBkwd(device_, axis_, &off, 1);
        } else if (home_direction_ == 1) {
            ECC_controlContinousFwd(device_, axis_, &on, 1);
            ECC_getStatusEotFwd(device_, axis_, &eot_found);
            start_time = ros::Time::now();
            while (eot_found != 1) {
                ECC_getStatusEotFwd(device_, axis_, &eot_found);
                if ((ros::Time::now() - start_time) > max_duration) {
                    ROS_WARN_STREAM("Finding EOT travel limit exceeded timeout");
                    break;
                }
                ros::Duration(0.05).sleep();
            }
            if (eot_found == 1) {
                ROS_INFO_STREAM("EOT limit for " << joint_name_ << " has been found");
            }
            ECC_controlContinousFwd(device_, axis_, &off, 1);

        } else {
            ROS_ERROR_STREAM(
                    "Direction was not 0 for backward or 1 for forward, set the direction to either of those values");
        }

        // Reenable output now the EOT has been found
        if (enableActor(true)) {
            ROS_INFO_STREAM("Actor for " << joint_name_ << " has been re-enabled");
        }

    } else {
        ROS_ERROR_STREAM("Actor for " << joint_name_
                                      << " joint failed to set reaching the EOT to deactivate the output with the error message: "
                                      << getECCErrorMessage(rc));
    }
    return eot_found;
}

bool AttocubeActor::setActorType(int type) {
    int rc;
    rc = ECC_controlActorSelection(device_, axis_,
                                   &type, 1);
    if(rc != NCB_Ok){
        ROS_ERROR_STREAM("Failed to set actor type with following error message:\n\t" << getECCErrorMessage(rc));
        return false;
    }

    return true;
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
    return (double) nano_metre / 1e9;
}

int toMicroDegree(double radian){
    return (int) (angles::to_degrees(radian) * 1e6);
}

double toRadian(int micro_degree){
    return (double) angles::from_degrees(micro_degree / 1e6);
}