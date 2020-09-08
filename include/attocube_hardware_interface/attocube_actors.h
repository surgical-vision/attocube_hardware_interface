//
// Created by george on 9/3/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H

#include <string>
#include <openacc.h>

#define ECSx5050    6;
#define ECGp5050    10;
#define ECR5050     14;

class AttocubeActor{
public:
    int device_;
    int axis_;
    std::string joint_name_;
    int frequency_; // frequency in mHz
    int amplitude_; // Amplitude in mV
    int actor_type_;

    AttocubeActor(int device, int axis, std::string joint_name, int actor_type) :
            device_(device), axis_(axis), joint_name_(std::move(joint_name)), actor_type_(actor_type) {};
};

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H
