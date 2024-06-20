//
// Created by You, Jisang on 04/03/2024.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class gimbal_commander : public rclcpp::Node {
public:
    gimbal_commander() : Node("gimbal_commander_node") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gimbal_command", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&gimbal_commander::publishMessage, this));
    }

    // void setWorld(raisim::World * world)
    // {
    //     virtualWorld_ = world;
    //     raibot = reinterpret_cast<raisim::ArticulatedSystem *>(virtualWorld_->getObject("robot"));  // robot state container
    //     active_control = true;  //active control uses the robot's state to help stabilize the gimbal
    // }


private:
    void publishMessage() {
        
        auto message = geometry_msgs::msg::Vector3();

        // if(active_control){
        //     // compute robot angular velocity in body frame
        //     raibot->getState(genCo_, genVel_);
        //     quat = genCo_.segment<4>(3);
        //     raisin::robot::sensor::quatToRotMat(quat, rot);
        //     angVelW = genVel_.segment<3>(3);    //world frame
        //     angVelB = rot.transpose() * angVelW; //body frame

        //     raisin::robot::sensor::rotMatToZYXEuler(rot, eulerAng);
        //     roll = eulerAng[0];
        //     pitch = eulerAng[1];

        //     message.x = -roll;
        //     message.y = -pitch-0.2;
        //     message.z = -0.2*angVelB[2];//0.9*sin(count/300.0);
        //     count++;
        // }
        // else{
        //     message.x = 0.0;
        //     message.y = -0.2;
        //     message.z = 0.9*sin(count/300.0);   //left and right motion for testing
        //     count++;
        // }

        message.x = 0.0;
        message.y = -0.2;
        message.z = 0.9*sin(count/300.0);   //left and right motion for testing
        count++;

        // Publish the message
        publisher_->publish(message);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::VectorXd genCo_, genVel_;
    Eigen::Vector4d quat;  // W --> B
    Eigen::Vector3d pos;   // W --> B
    Eigen::Vector3d linVelW, angVelW, angVelB, linAccB;
    Eigen::Matrix3d rot;  // W --> B
    Eigen::Vector3d eulerAng;
    double roll, pitch;
    int count = 0;
    bool active_control = false;
    // raisim::World * virtualWorld_;
    // raisim::ArticulatedSystem * raibot;
};