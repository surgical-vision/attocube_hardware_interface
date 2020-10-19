//
// Created by george on 10/19/20.
//

#include <attocube_hardware_interface/attocube_actors.h>

AttocubeActor::AttocubeActor(int device, int axis, std::string joint_name, int actor_type) :
        device_(device), axis_(axis), joint_name_(std::move(joint_name)), actor_type_(actor_type) {
    amplitude_ = 40333;
    frequency_ = 2222222;
};

int* AttocubeActor::getType(){
    return &actor_type_;
}


double AttocubeActor::estimateVelocity(){
    getRawCurrentPosition();
    double velocity;
    if (actor_type_ == ECC_actorLinear){
        velocity = toMetre(current_position_ - previous_position_) / (current_read_time_ - previous_read_time_).toSec();
    } else{
        velocity = toRadian(current_position_ - previous_position_) / (current_read_time_ - previous_read_time_).toSec();
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
    if(rc != NCB_Ok){
        return false;
    }
    return true;
}

bool AttocubeActor::setDesiredPosition(double desired_position) {
    if(actor_type_ == ECC_actorLinear) {
        return setRawDesiredPosition(toNanoMetre(desired_position));
    } else{
        return setRawDesiredPosition(toMicroDegree(desired_position));
    }
}

bool AttocubeActor::setActorFrequency(int frequency) {
    frequency_ = frequency;
    int rc;
    rc = ECC_controlFrequency(device_, axis_, &frequency_, 1);
    if(rc != NCB_Ok){
        return false;
    }
    return true;
}

bool AttocubeActor::setActorAmplitude(int amplitude) {
    amplitude_ = amplitude;
    int rc;
    rc = ECC_controlAmplitude(device_, axis_, &amplitude_, 1);
    if(rc != NCB_Ok){
        return false;
    }
    return true;
}

bool AttocubeActor::enableActor(bool on) {
    int val = (int)on;
    int rc = 0;

    // Set the current position as the desired position;
    setRawDesiredPosition(getRawCurrentPosition());

    rc = ECC_controlOutput(device_, axis_, &val, 1);
    if (rc != NCB_Ok){
        return false;
    }
    rc = ECC_controlMove(device_, axis_, &val, 1);
    if (rc != NCB_Ok){
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
    if(rc != NCB_Ok){
        return NAN;
    }
    return current_position_;
}

double AttocubeActor::getCurrentPosition() {
    if(actor_type_ == ECC_actorLinear) {
        return toMetre(getRawCurrentPosition());
    } else{
        return toRadian(getRawCurrentPosition());
    }
    return 0;
}

double AttocubeActor::getCurrentVelocity() {
    return 0;
}

bool AttocubeActor::findEOTLimits(int timeout) {
    return false;
}
