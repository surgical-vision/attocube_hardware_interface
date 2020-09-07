//
// Created by george on 9/3/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H

#include <string>

class AttocubeActor{
    int device_;
    int axis_;
    std::string joint_name_;
    int frequency_; // frequency in mHz
    int amplitude_; // Amplitude in mV
};

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H
