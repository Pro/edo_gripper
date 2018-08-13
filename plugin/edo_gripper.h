//
// Created by profanter on 8/9/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_EDOGRIPPER_H
#define PROJECT_EDOGRIPPER_H


// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
    class EdoGripper : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);



        static gazebo::physics::LinkPtr getLinkEndingWith(physics::ModelPtr model, std::string link);
        static gazebo::physics::JointPtr getJointEndingWith(physics::ModelPtr model, std::string link);

    public:
        void OnUpdate();

        // Called on world reset
        virtual void Reset();

    private:
        /// Pointer to the model
        physics::ModelPtr model;
        ///name of the gps and the communication channel
        std::string name;

        transport::NodePtr node;

        ///Suscriber for set_gripper_span
        ros::Subscriber set_gripper_span_subscriber;

        ros::Time last_update_sim_time_ros;

        ///Set gripper callback
        void on_set_gripper_span_msg(const std_msgs::Float32ConstPtr &msg);

        /// Publisher for gripper_span
        ros::Publisher gripper_span_publisher;

        float desired_span;
        float current_span;

        ros::NodeHandle model_nh;

        /// Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        gazebo::physics::JointPtr gripper_left_base_joint;
        gazebo::physics::JointPtr gripper_left_finger_joint;
        gazebo::physics::JointPtr gripper_right_base_joint;
        gazebo::physics::JointPtr gripper_right_finger_joint;

        void setGripperSpan(float span);
    };

    /// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(EdoGripper)
}

#endif //PROJECT_EDOGRIPPER_H
