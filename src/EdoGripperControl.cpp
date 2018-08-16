//
// Created by profanter on 8/14/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include "EdoGripperControl.h"

#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <EdoGripperControl.h>


EdoGripperControl::EdoGripperControl(): nh() {

    //create subscriber
    this->set_gripper_span_subscriber = nh.subscribe(std::string("set_gripper_span"), 1,
                                                           &EdoGripperControl::on_set_gripper_span_msg, this);
    this->joint_states_subscriber = nh.subscribe(std::string("joint_states"), 1,
                                                           &EdoGripperControl::on_joint_state_msg, this);

    this->gripper_span_publisher = nh.advertise<std_msgs::Float32>("edo_gripper_span",1);

    this->gripper_state_publisher = nh.advertise<std_msgs::Int8>("edo_gripper_state",1);

    std::string controllerName;
    nh.param<std::string>("gripper_left_base_controller", controllerName, "edo_gripper_left_base_controller");
    this->gripper_left_base_pub = nh.advertise<std_msgs::Float64>(controllerName + "/command",1);
        nh.param<std::string>("gripper_left_finger_controller", controllerName, "edo_gripper_left_finger_controller");
    this->gripper_left_finger_pub = nh.advertise<std_msgs::Float64>(controllerName + "/command",1);
    nh.param<std::string>("gripper_right_base_controller", controllerName, "edo_gripper_right_base_controller");
    this->gripper_right_base_pub = nh.advertise<std_msgs::Float64>(controllerName + "/command",1);
        nh.param<std::string>("gripper_right_finger_controller", controllerName, "edo_gripper_right_finger_controller");
    this->gripper_right_finger_pub = nh.advertise<std_msgs::Float64>(controllerName + "/command",1);
}

// Called by the world update start event
void EdoGripperControl::OnUpdate() {
    // Apply a small linear velocity to the model.
    //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

    std_msgs::Float32 msg;
    msg.data = static_cast<float>(current_span);
    this->gripper_span_publisher.publish(msg);


    gripper_moving = abs(current_span - desired_span) > 0.001;

    if (gripper_moving)
        this->setGripperSpan(desired_span);

    std_msgs::Int8 msgState;
    msgState.data = gripper_moving;
    this->gripper_state_publisher.publish(msgState);



}

/** Functions for recieving Messages (registerd via suscribers)
 * @param msg message
 */
void EdoGripperControl::on_set_gripper_span_msg(const std_msgs::Float32ConstPtr &msg) {
    if (msg->data <= 0)
        desired_span = 0;
    else if (msg->data >= 0.08)
        desired_span = 0.08;
    else
        desired_span = msg->data;

    setGripperSpan(desired_span);
}

// Called on world reset
void EdoGripperControl::Reset()
{
    // 50mm
    desired_span = 0.05;
    setGripperSpan(desired_span);
}

void EdoGripperControl::setGripperSpan(double span) {

    ROS_DEBUG_NAMED("gazebo_edo_gripper", "Set span: %f", desired_span);

    this->current_span = span;

    span *= 1000;

    // this magic formula comes from manually measuring the distance and angle and create a trend line using excel
    double fingerbase_angle = -0.7428 * span + 29.72;
    double fingertip_angle = -fingerbase_angle;

    fingertip_angle = fingertip_angle / 180 * M_PI;
    fingerbase_angle = fingerbase_angle / 180 * M_PI;

    std_msgs::Float64 fingerMsg;
    fingerMsg.data = fingertip_angle;
    std_msgs::Float64 baseMsg;
    baseMsg.data = fingerbase_angle;

    gripper_left_finger_pub.publish(fingerMsg);
    gripper_left_base_pub.publish(baseMsg);
    gripper_right_finger_pub.publish(fingerMsg);
    gripper_right_base_pub.publish(baseMsg);

}

void EdoGripperControl::on_joint_state_msg(const sensor_msgs::JointStateConstPtr &msg) {
    // we are only looking for one of the current angles for the base joint

    double base_angle = 0;
    bool found = false;
    for (unsigned long i=0; i< msg->position.size(); i++) {
        if (msg->name.at(i) == "edo_gripper_left_base_joint") {
            base_angle = msg->position.at(i);
            found = true;
        }
    }
    if (!found) {
        ROS_WARN_ONCE_NAMED("edo_gripper_controller", "The joint state message does not contain 'edo_gripper_left_base_joint'. This message will only print once.");
        return;
    }

    base_angle = base_angle / M_PI * 180.0;
    current_span = (-(base_angle - 29.72)/0.7428) / 1000;
}
