//
// Created by george on 9/11/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_UTILS_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_UTILS_H

#include <angles/angles.h>

static std::string getECCErrorMessage( int code )
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

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_UTILS_H
