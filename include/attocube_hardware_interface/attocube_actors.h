//
// Created by george on 9/3/20.
//

#ifndef ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H
#define ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H

#include <string>
#include <angles/angles.h>
#include <openacc.h>
#include <ros/ros.h>
#include <ecc.h>
#include <attocube_hardware_interface/attocube_utils.h>

#define ECSx5050    6
#define ECGp5050    10
#define ECR5050     14


class AttocubeActor{
public:
    int device_;
    int axis_;
    std::string joint_name_;
    int frequency_; // frequency in mHz
    int amplitude_; // Amplitude in mV
    int actor_type_;
    int current_position_;
    int previous_position_;
    int desired_position_;
    bool reference_valid_;
    int refernce_position_;
    ros::Time previous_read_time_;
    ros::Time current_read_time_;
    int home_direction_ = 0;
    int target_range = 10;

    AttocubeActor(int device, int axis, std::string joint_name, int actor_type);
    ~AttocubeActor();

    // Setters
    /** @brief set desired position in raw units nanometre or microrad
     * set desired position of actor in in raw units nanometre or microrad
     * @param desired_position: position in nanometre or micro deg
     * @return result of function
     */
    bool setRawDesiredPosition(int desired_position);

    /** @brief set desired position in SI units
     * set desired position of actor in SI units according to the actor type metres or radians
     * @param desired_position: position in metre or radian
     * @return result of function
     */
    bool setDesiredPosition(double desired_position);

    /** @brief set actor frequency
     * set frequency of the actuator signal
     * @param frequency: frequency in mHz
     * @return result of function
     */
    bool setActorFrequency(int frequency);

    /** @brief set actor amplitude
     * set amplitude of the actuator signal
     * @param amplitude: amplitude in mV
     * @return result of function
     */
    bool setActorAmplitude(int amplitude);

    /** @brief enables the actor
     * sets the output relay for the selected axis
     * @param on: boolean for on or off
     * @return result of function
     */
    bool enableActor(bool on);

    // Getters
    /** @brief actor type
     * gets the actor type eg: linear, goniometer, rotation
     * @return: Actor type as enum ECC_actorType
     */
    int* getType();

    /** @brief raw actor position
     * returns the current position in raw units nanometre or microrad
     * @return: Current position in raw units (nm or micro deg)
     */
    int getRawCurrentPosition();

    /** @brief actor position
     * returns the current position in SI units depending on actor type
     * @return: Current position in SI units (m or rad)
     */
    double getCurrentPosition();

    /** @brief actor velocity
     * returns the current velocity in SI units depending on actor type
     * @return: Current velocity in SI units (m/s or rad/s)
     */
    double estimateVelocity();

    /** @brief set actor type
     * Sets the actor type within a member variable and sends to the controller
     * @param type: defined in the attopcube_actors.h
     * @return
     */
    bool setActorType(int type);

    // Utils
    bool findEOTLimits(int timeout);
};

std::string getECCErrorMessage( int code );

int toNanoMetre(double metre);

double toMetre(int nano_metre);

int toMicroDegree(double radian);

double toRadian(int micro_degree);

#endif //ATTOCUBE_HARDWARE_INTERFACE_ATTOCUBE_ACTORS_H
