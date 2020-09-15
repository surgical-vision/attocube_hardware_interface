//
// Created by george on 9/11/20.
//

#include <attocube_hardware_interface/attocube_ros_interface.h>

AttocubeRosInterface::AttocubeRosInterface(ros::NodeHandle &nh) : interface_(nh) {
    nh_ = nh;
    publisher_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    subscriber_joint_trajectory_ = nh_.subscribe("command_trajectory", 10, &AttocubeRosInterface::callbackJointTrajectory, this);
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeRosInterface::callbackSrvEnableActors, this);
    service_reset_actors_ = nh_.advertiseService("reset_actors", &AttocubeRosInterface::callbackSrvResetActors, this);
    service_home_actors_ = nh_.advertiseService("home_actors", &AttocubeRosInterface::callbackSrvHomeActors, this);
}

void AttocubeRosInterface::generateJointStateMsg(sensor_msgs::JointState &msg) {
    msg.name.clear();
    msg.effort.clear();
    msg.position.clear();
    msg.velocity.clear();

    for(auto& actor : interface_.actors_){
        msg.name.push_back(actor.first);
        if (actor.second.actor_type_ == ECC_actorLinear){
            msg.position.push_back(toMetre(actor.second.current_position_));
            msg.velocity.push_back(actor.second.estimateVelocity());
        } else{
            msg.position.push_back(toRadian(actor.second.current_position_));
            msg.velocity.push_back(actor.second.estimateVelocity());
        }
    }
}

bool AttocubeRosInterface::readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    //Check if there are multiple points
    if(msg->points.size() > 1){
        ROS_WARN_STREAM("Found multiple trajectory points\nCurrently only supports a single desired position for each joint\nOnly the first point will be used");
    }
    for(int i = 0; i < msg->joint_names.size(); i++){
        auto actor = interface_.actors_.find(msg->joint_names[i]);
        if(actor != interface_.actors_.end()){
            // Found the actor with the joint name;

            if(actor->second.actor_type_ == ECC_actorLinear) {
                actor->second.desired_position_ = toNanoMetre(msg->points[0].positions[i]);
                ROS_DEBUG_STREAM(actor->first << " moving to " << actor->second.desired_position_ << " nm");
            } else{
                actor->second.desired_position_ = toMicroDegree(msg->points[0].positions[i]);
                ROS_DEBUG_STREAM(actor->first << " moving to " << actor->second.desired_position_ << " micro deg");
            }
        } else{
            ROS_WARN_STREAM(msg->joint_names[i] << " is not assigned to an actor");
        }
    }
    return true;
}

void AttocubeRosInterface::callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    ROS_INFO_STREAM("Received a Joint trajectory");
    readJointTrajectoryMsg(msg);
    interface_.writePositions();
}

bool AttocubeRosInterface::callbackSrvEnableActors(std_srvs::SetBool::Request &request,
                                                   std_srvs::SetBool::Response &response) {
    ROS_INFO_STREAM("Enabling actors");
    bool on = request.data;
    response.success = interface_.enableActors(on);
    return response.success;
}

void AttocubeRosInterface::publishJointState() {
    sensor_msgs::JointState js;
    interface_.readPositions();
    generateJointStateMsg(js);
    publisher_joint_state_.publish(js);
}

bool AttocubeRosInterface::callbackSrvResetActors(std_srvs::Trigger::Request &request,
                                                  std_srvs::Trigger::Response &response) {
    ROS_INFO_STREAM("Reseting all actors positions\nFollow this with homing");
    response.success = interface_.resetPositions();
    return response.success;
}

bool AttocubeRosInterface::callbackSrvHomeActors(std_srvs::Trigger::Request &request,
                                                 std_srvs::Trigger::Response &response) {
    ROS_INFO_STREAM("Homing all actors now");
    response.success = interface_.homeAllActors();
    return response.success;
}

bool AttocubeRosInterface::hardcodeSetupDevice() {
    if (interface_.getDevicesAvailable() > 1) {
        interface_.setupDevices();
        ROS_INFO_STREAM("Devices setup");
        interface_.getHardcodedConfig();
        ROS_INFO_STREAM("Setting up actors");
        interface_.setupActors();
        return true;
    } else{
        ROS_ERROR_STREAM("No devices available");
        return false;
    }
}

int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh;
    AttocubeRosInterface communication(nh);
    if(communication.hardcodeSetupDevice()){
        ros::Rate rate(100);
        while (ros::ok()){
            communication.publishJointState();
            ROS_INFO_STREAM_THROTTLE(1, "Current Cylcle Time: " << rate.cycleTime());
            ros::spinOnce();
            rate.sleep();
        }
        return 0;
    } else{
        ROS_ERROR_STREAM("Devices were not setup properly");
        return -1;
    }
}
