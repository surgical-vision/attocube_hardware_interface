//
// Created by george on 9/11/20.
//

#include <attocube_hardware_interface/attocube_ros_interface.h>

// AttocubeRosInterface::AttocubeRosInterface(ros::NodeHandle &nh) : interface_(nh), action_follow_joint_trajectory_(nh_, "FollowJointTrajectory", boost::bind(&AttocubeRosInterface::callbackExecuteFollowJointTrajectory, this, _1), false) {
AttocubeRosInterface::AttocubeRosInterface(ros::NodeHandle &nh) : interface_(nh) {
    nh_ = nh;
    publisher_joint_state_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    subscriber_joint_trajectory_ = nh_.subscribe("command_trajectory", 10, &AttocubeRosInterface::callbackJointTrajectory, this);
    service_enable_actors_ = nh_.advertiseService("enable_actors", &AttocubeRosInterface::callbackSrvEnableActors, this);
    service_reset_actors_ = nh_.advertiseService("reset_actors", &AttocubeRosInterface::callbackSrvResetActors, this);
    service_home_actors_ = nh_.advertiseService("home_actors", &AttocubeRosInterface::callbackSrvHomeActors, this);
    service_trigger_ros_control_ = nh_.advertiseService("enable_ros_control", &AttocubeRosInterface::callbackSrvStartROSControl, this)
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

void AttocubeRosInterface::register_interfaces() {
    // Check the actors have been configured, enabled and homed before adding them as a controllable interface
    if(!interface_.actors_.empty() && interface_.allActorsEnabled() && interface_.allActorsReferenced()){
        int i = 0;
        current_position_.assign(interface_.actors_.size(), 0);
        current_velocity_.assign(interface_.actors_.size(), 0);
        command_position_.assign(interface_.actors_.size(), 0);
        for(auto &actor : interface_.actors_){
            // For each actor add the state variables to the state handle and desired to the position interface
            hardware_interface::JointStateHandle state_handle(actor.first,
                                                              &(current_position_[i]),
                                                              &(current_velocity_[i]),
                                                              &(effort_placeholder_));
            jnt_state_interface.registerHandle(state_handle);

            hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
                    jnt_state_interface.getHandle(actor.first), &command_position_[i]);

            jnt_pos_interface.registerHandle(position_joint_handle);

            // TODO: Add joint limits from URDF

            i++;
        }
        registerInterface(&jnt_pos_interface);
        registerInterface(&jnt_state_interface);
    }
}

void AttocubeRosInterface::read(ros::Duration duration) {
    interface_.readPositions();
    int i = 0;
    for(auto &actor : interface_.actors_){
        current_position_[i] = actor.second.current_position_;
        current_velocity_[i] = actor.second.estimateVelocity();
        i++;
    }
}

void AttocubeRosInterface::write(ros::Duration duration) {
    int i = 0;
    for(auto &actor : interface_.actors_){
        if(actor.second.actor_type_ == ECC_actorLinear) {
            actor.second.desired_position_ = toNanoMetre(command_position_[i]);
            ROS_DEBUG_STREAM(actor.first << " moving to " << actor.second.desired_position_ << " nm");
        } else{
            actor.second.desired_position_ = toMicroDegree(command_position_[i]);
            ROS_DEBUG_STREAM(actor.first << " moving to " << actor.second.desired_position_ << " micro deg");
        }
        i++;
    }
    interface_.writePositions();

}

bool AttocubeRosInterface::callbackSrvStartROSControl(std_srvs::SetBool::Request &request,
                                                      std_srvs::SetBool::Response &response) {
    // Check if requesting start or stop
    // Start request
    if(request.data == true){
        // Check if the actors are enabled and referenced
        bool enabled = interface_.allActorsEnabled();
        bool referenced = interface_.allActorsReferenced();
        if(referenced && enabled){
            enabled_ros_control = true;
        }
    }
    //Stop request
    else{
        enabled_ros_control = false;
    }
    return true;
}

int main( int argc, char ** argv ) {
    ros::init(argc, argv, "attocube_hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    AttocubeRosInterface communication(nh);

    controller_manager::ControllerManager cm(&communication, nh);
    if(communication.hardcodeSetupDevice()){
        communication.register_interfaces();
        ros::Time current_time, previous_time = ros::Time::now();
        ros::Duration elapsed_time;

        while (ros::ok()){
            current_time = ros::Time::now();
            elapsed_time = ros::Duration(current_time - previous_time);
            previous_time = current_time;

            communication.read(elapsed_time);
            cm.update(current_time, elapsed_time);
            communication.write(elapsed_time);

        }
        return 0;
    } else{
        ROS_ERROR_STREAM("Devices were not setup properly");
        return -1;
    }
}
