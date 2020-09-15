//
// Created by george on 9/11/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_UTILS_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_UTILS_H

#include <angles/angles.h>
#include <ecc.h>
#include <string>

std::string getECCErrorMessage( int code );

int toNanoMetre(double metre);

double toMetre(int nano_metre);

int toMicroDegree(double radian);

double toRadian(int micro_degree);

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_UTILS_H
