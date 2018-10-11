//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include "ros/ros.h"

#include "EdoGripperControl.h"

#define DEG_TO_RAD (M_PI/180.0)

int main(int argc, char **argv) {

    ros::init(argc, argv, "edo_gripper_node");

    ros::NodeHandle n("~");

    int update_rate;
    n.param<int>("update_rate", update_rate, 50);

    EdoGripperControl control;
    control.Reset();

    ros::Rate loop_rate(update_rate);


    while(ros::ok())
    {
        control.OnUpdate();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}