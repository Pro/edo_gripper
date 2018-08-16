//
// Created by profanter on 8/14/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_EDOGRIPPERCONTROL_H
#define PROJECT_EDOGRIPPERCONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

class EdoGripperControl {

public:
    explicit EdoGripperControl();
    virtual ~EdoGripperControl() = default;

    void OnUpdate();

    virtual void Reset();

private:


    ///Suscriber for set_gripper_span
    ros::Subscriber set_gripper_span_subscriber;
    ros::Subscriber joint_states_subscriber;

    ///Set gripper callback
    void on_set_gripper_span_msg(const std_msgs::Float32ConstPtr &msg);

    void on_joint_state_msg(const sensor_msgs::JointStateConstPtr &msg);

    /// Publisher for gripper_span
    ros::Publisher gripper_span_publisher;
    ros::Publisher gripper_state_publisher;

    ros::Publisher gripper_left_base_pub;
    ros::Publisher gripper_left_finger_pub;
    ros::Publisher gripper_right_base_pub;
    ros::Publisher gripper_right_finger_pub;

    double desired_span;
    double current_span;

    bool gripper_moving;

    ros::NodeHandle nh;

    void setGripperSpan(double span);

};


#endif //PROJECT_EDOGRIPPERCONTROL_H
