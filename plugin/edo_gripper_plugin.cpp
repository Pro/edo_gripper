//
// Created by profanter on 8/9/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include "edo_gripper.h"

#include <ros/ros.h>

using namespace gazebo;


void EdoGripper::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf) {
    current_span = 0;
    desired_span = 0;
    gripper_moving = false;

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // Store the pointer to the model
    this->model = _parent;

    this->name = model->GetName();
    ROS_INFO_NAMED("gazebo_edo_gripper", "Loading e.DO Gripper Plugin of model %s\n", name.c_str());

    std::string robot_namespace_;

    // Get namespace for nodehandle
    if(sdf->HasElement("robotNamespace"))
    {
        robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    else
    {
        robot_namespace_ = _parent->GetName(); // default
    }

    // Get robot_description ROS param name
    /*if (sdf->HasElement("robotParam"))
    {
        robot_description_ = sdf->GetElement("robotParam")->Get<std::string>();
    }
    else
    {
        robot_description_ = "robot_description"; // default
    }*/

    // Get parameters/settings for controllers from ROS param server
    model_nh = ros::NodeHandle(robot_namespace_);
//    for (int i = 0; i < model->GetJointCount(); i++) {
//        ROS_INFO("Model joint: %s", model->GetJoints()[i]->GetName().c_str());
//    }

    ROS_INFO_NAMED("gazebo_edo_gripper", "Starting gazebo_edo_gripper plugin in namespace: %s", robot_namespace_.c_str());


    gripper_left_base_joint = getJointEndingWith(this->model, "gripper_left_base_joint");
    if (gripper_left_base_joint == nullptr) {
        ROS_ERROR_NAMED("gazebo_edo_gripper", "Joint ending with 'gripper_left_base_joint' not found in model");
    }
    gripper_left_finger_joint = getJointEndingWith(this->model, "gripper_left_finger_joint");
    if (gripper_left_finger_joint == nullptr) {
        ROS_ERROR_NAMED("gazebo_edo_gripper", "Joint ending with 'gripper_left_finger_joint' not found in model");
    }
    gripper_right_base_joint = getJointEndingWith(this->model, "gripper_right_base_joint");
    if (gripper_right_base_joint == nullptr) {
        ROS_ERROR_NAMED("gazebo_edo_gripper", "Joint ending with 'gripper_right_base_joint' not found in model");
    }
    gripper_right_finger_joint = getJointEndingWith(this->model, "gripper_right_finger_joint");
    if (gripper_right_finger_joint == nullptr) {
        ROS_ERROR_NAMED("gazebo_edo_gripper", "Joint ending with 'gripper_right_finger_joint' not found in model");
    }


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&EdoGripper::OnUpdate, this));

    //Create the communication Node for communication with fawkes
    this->node = transport::NodePtr(new transport::Node());
    //the namespace is set to the model name!
    this->node->Init(model->GetWorld()->Name() + "/" + name);

    //create subscriber
    this->set_gripper_span_subscriber = model_nh.subscribe(std::string("set_gripper_span"), 1,
                                                              &EdoGripper::on_set_gripper_span_msg, this);

    this->gripper_span_publisher = model_nh.advertise<std_msgs::Float32>("gripper_span",1);

    this->gripper_state_publisher = model_nh.advertise<std_msgs::Int8>("gripper_state",1);

    Reset();
}

// Called by the world update start event
void EdoGripper::OnUpdate() {
    // Apply a small linear velocity to the model.
    //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

    std_msgs::Float32 msg;
    msg.data = current_span;
    this->gripper_span_publisher.publish(msg);

    std_msgs::Int8 msgState;
    msgState.data = gripper_moving;
    this->gripper_state_publisher.publish(msgState);

    // Get the simulation time and period
    gazebo::common::Time gz_time_now = model->GetWorld()->SimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros;


    if (sim_period.toSec() < 0.1)
        // limit update rate
        return;

    last_update_sim_time_ros = sim_time_ros;

    // we want to move 30mm per sec:
    float mPerSec = 0.03;

    float diffMm = current_span - desired_span;

    /*if (abs(diffMm) < 0.001) {
        // skip if difference is smaller than 1mm
        return;
    }*/

    float maxAllowedMove = static_cast<float>(mPerSec * sim_period.toSec());

    if (diffMm < 0) {
        // we need to open
        if (current_span + maxAllowedMove > desired_span)
            setGripperSpan(desired_span);
        else
            setGripperSpan(current_span + maxAllowedMove);
    } else {
        // we need to close
        if (current_span - maxAllowedMove < desired_span)
            setGripperSpan(desired_span);
        else
            setGripperSpan(current_span - maxAllowedMove);
    }

    if (abs(diffMm) < 0.001) {
        gripper_moving = false;
    }

}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void EdoGripper::on_set_gripper_span_msg(const std_msgs::Float32ConstPtr &msg) {
    if (msg->data <= 0)
        desired_span = 0;
    else if (msg->data >= 0.08)
        desired_span = 0.08;
    else
        desired_span = msg->data;

    gripper_moving = true;

    ROS_DEBUG_NAMED("gazebo_edo_gripper", "Set span: %f", desired_span);
}

// Called on world reset
void EdoGripper::Reset()
{
    // Reset timing variables to not pass negative update periods to controllers on world reset
    last_update_sim_time_ros = ros::Time();

    // 50mm
    desired_span = 0.05;
    setGripperSpan(0.05);
}

void EdoGripper::setGripperSpan(float span) {


    if (gripper_left_base_joint == nullptr || gripper_left_finger_joint == nullptr ||
            gripper_right_base_joint == nullptr || gripper_right_finger_joint == nullptr) {
        return;
    }

    this->current_span = span;

    span *= 1000;

    double fingerbase_angle = -(0.7428 * span + 150.28)+180;
    double fingertip_angle = -fingerbase_angle + 180;

    fingertip_angle = fingertip_angle / 180 * M_PI;
    fingerbase_angle = fingerbase_angle / 180 * M_PI;

    /*gripper_left_base_joint->SetPosition(0, fingerbase_angle, false);
    gripper_right_base_joint->SetPosition(0, fingerbase_angle, false);
    gripper_left_finger_joint->SetPosition(0, fingertip_angle, false);
    gripper_right_finger_joint->SetPosition(0, fingertip_angle, false);*/

}


inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

gazebo::physics::LinkPtr EdoGripper::getLinkEndingWith(physics::ModelPtr model, std::string ending) {
    std::vector<gazebo::physics::LinkPtr> links = model->GetLinks();
    for (unsigned int i=0; i<links.size(); i++) {
        if (ends_with(links[i]->GetName(), ending))
            return links[i];
    }
    return nullptr;
}

gazebo::physics::JointPtr EdoGripper::getJointEndingWith(physics::ModelPtr model, std::string ending) {
    std::vector<gazebo::physics::JointPtr> joints = model->GetJoints();
    for (unsigned int i=0; i<joints.size(); i++) {
        if (ends_with(joints[i]->GetName(), ending))
            return joints[i];
    }
    return nullptr;
}