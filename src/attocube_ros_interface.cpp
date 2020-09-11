//
// Created by george on 9/11/20.
//

#include <attocube_hardware_interface/attocube_ros_interface.h>

AttocubeRosInterface::AttocubeRosInterface(ros::NodeHandle &nh) : interface_(nh) {
    nh_ = nh;
    publisher_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    subscriber_joint_trajectory_ = nh_.subscribe("command_trajectory", 10, &AttocubeRosInterface::callbackJointTrajectory, this);
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeRosInterface::callbackSrvEnableActors, this);
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

bool AttocubeRosInterface::readJointTrajectoryMsg(
        const trajectory_msgs::JointTrajectory_<std::allocator<void>>::ConstPtr &msg) {
    //Check if there are multiple points
    if(msg->points.size() > 1){
        ROS_WARN_STREAM("Found multiple trajectory points\nCurrently only supports a single desired position for each joint\nOnly the first point will be used");
    }
    for(int i = 0; i < msg->joint_names.size(); i++){
        auto actor = interface_.actors_.find(msg->joint_names[i]);
        if(actor != interface_.actors_.end()){
            // Found the actor with the joint name;
            if(actor->second.actor_type_ == ECC_actorLinear) {
                actor->second.desired_position_ = toNanoMetre(msg->points[i].positions[0]);
            } else{
                actor->second.desired_position_ = toMicroDegree(msg->points[i].positions[0]);
            }
        } else{
            ROS_WARN_STREAM(msg->joint_names[i] << " is not assigned to an actor");
        }
    }
    return true;
}

void AttocubeRosInterface::callbackJointTrajectory(
        const trajectory_msgs::JointTrajectory_<std::allocator<void>>::ConstPtr &msg) {
    ROS_INFO_STREAM("Received a Joint trajectory");
    readJointTrajectoryMsg(msg);
    interface_.writePositions();
}

bool AttocubeRosInterface::callbackSrvEnableActors(std_srvs::SetBool::Request &request,
                                                   std_srvs::SetBool::Response &response) {
    bool on = request.data;
    response.success = interface_.enableActors(on);
    return response.success;
}

int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh;
    AttocubeRosInterface communication(nh);
    return 0;
}
