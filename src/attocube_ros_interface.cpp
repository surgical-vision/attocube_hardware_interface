//
// Created by george on 9/11/20.
//

#include <attocube_hardware_interface/attocube_ros_interface.h>

AttocubeRosInterface::AttocubeRosInterface(ros::NodeHandle &nh) : interface_(nh), action_follow_joint_trajectory_(nh_, "FollowJointTrajectory", boost::bind(&AttocubeRosInterface::callbackExecuteFollowJointTrajectory, this, _1), false) {
    nh_ = nh;
    publisher_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    subscriber_joint_trajectory_ = nh_.subscribe("command_trajectory", 10, &AttocubeRosInterface::callbackJointTrajectory, this);
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeRosInterface::callbackSrvEnableActors, this);
    service_reset_actors_ = nh_.advertiseService("reset_actors", &AttocubeRosInterface::callbackSrvResetActors, this);
    service_home_actors_ = nh_.advertiseService("home_actors", &AttocubeRosInterface::callbackSrvHomeActors, this);
    service_enable_action_trajectory_ = nh_.advertiseService("enable_action_trajectory", &AttocubeRosInterface::callbackSrvEnableActionTrajectory, this);

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

bool AttocubeRosInterface::readJointTrajectoryMsg(const trajectory_msgs::JointTrajectory::ConstPtr &msg, int point_index) {
    for(int i = 0; i < msg->joint_names.size(); i++){
        auto actor = interface_.actors_.find(msg->joint_names[i]);
        if(actor != interface_.actors_.end()){
            // Found the actor with the joint name;

            if(actor->second.actor_type_ == ECC_actorLinear) {
                actor->second.desired_position_ = toNanoMetre(msg->points[point_index].positions[i]);
                ROS_DEBUG_STREAM(actor->first << " moving to " << actor->second.desired_position_ << " nm");
            } else{
                actor->second.desired_position_ = toMicroDegree(msg->points[point_index].positions[i]);
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
    //Check if there are multiple points
    if(msg->points.size() > 1){
        ROS_WARN_STREAM("Found multiple trajectory points\nCurrently only supports a single desired position for each joint\nOnly the first point will be used");
    }
    readJointTrajectoryMsg(msg, 0);
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

void AttocubeRosInterface::callbackExecuteFollowJointTrajectory(control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
    int num_of_points = goal->trajectory.points.size();
    int current_point = 0;
    bool motion_sent = false, motion_finished = false;

//    trajectory_msgs::JointTrajectory::Ptr trajectory = goal->trajectory);
    // while not preempted and ros is ok
    // Send the trajectory point to motioncontroller and wait for it to achieve the position
    while(!action_follow_joint_trajectory_.isPreemptRequested() && ros::ok() && current_point < num_of_points){
        if(!motion_sent){
//            readJointTrajectoryMsg(trajectory, current_point);
            interface_.writePositions();
            motion_sent = true;
        }


    }
    // Find out what broke the loop

}

bool AttocubeRosInterface::callbackSrvEnableActionTrajectory(std_srvs::SetBool::Request &request,
                                                             std_srvs::SetBool::Response &response) {
    //Check if activating it or disabling it
    if(request.data != 0) {
        // Check if the action server is capable of running - actors enabled and position references valid
        if (!interface_.allActorsEnabled()) {
            response.message = "Actors are not initialised, initialise the actors before enabling the action sever";
            ROS_WARN_STREAM(response.message);
            response.success = false;
            return response.success;
        }
        if (!interface_.allActorsReferenced()) {
            response.message = "Actors are not referenced, home the actors before enabling the action sever";
            ROS_WARN_STREAM(response.message);
            response.success = false;
            return response.success;
        }
        action_follow_joint_trajectory_.start();
    } else{
        //Disabling the action server
        result_follow_joint_trajectory_.error_code = result_follow_joint_trajectory_.INVALID_GOAL;
        result_follow_joint_trajectory_.error_string = "Goal aborted as action server has been set to shut down";
        action_follow_joint_trajectory_.setAborted(result_follow_joint_trajectory_);
        action_follow_joint_trajectory_.shutdown();
    }
    return true;
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
