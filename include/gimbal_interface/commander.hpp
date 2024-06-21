//
// Created by You, Jisang on 04/03/2024.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gimbal_interface/msg/robotstate.hpp" 

class gimbal_commander : public rclcpp::Node {
public:
    gimbal_commander() : Node("gimbal_commander_node") {

        subscriber_ = create_subscription<gimbal_interface::msg::robotstate>(
                "railab_raibo/gimbal_command", 10, std::bind(&gimbal_com::sendCommand, this, std::placeholders::_1));

        publisher_ = create_publisher<geometry_msgs::msg::Vector3>("/gimbal_command", 10);
        //timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&gimbal_commander::publishMessage, this));
    }

private:
    void sendCommand(const gimbal_interface::msg::robotstate state) {

        // compute robot angular velocity in body frame
        quat[0] = state.quaternion.w;
        quat[1] = state.quaternion.x;
        quat[2] = state.quaternion.y;
        quat[3] = state.quaternion.z;

        angVelW[0] = state.angVelW.x;
        angVelW[1] = state.angVelW.y;
        angVelW[2] = state.angVelW.z;

        rot = quat.toRotationMatrix();

        angVelB = rot.transpose() * angVelW; //body frame

        eulerAng = rot.eulerAngles(0, 1, 2);
        roll = eulerAng[0];
        pitch = eulerAng[1];

        auto message = geometry_msgs::msg::Vector3();
        message.x = -roll;
        message.y = -pitch-0.2;
        message.z = -0.2*angVelB[2];//0.9*sin(count/300.0);

        // for testing
        // message.x = 0.0;
        // message.y = -0.2;
        // message.z = 0.9*sin(count/300.0);   //left and right motion for testing
        // count++;

        // Publish the message
        publisher_->publish(message);
    }

    rclcpp::Subscription<gimbal_interface::msg::robotstate>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Quaterniond quat;  // W --> B
    Eigen::Vector3d angVelW, angVelB;
    Eigen::Matrix3d rot;  // W --> B
    Eigen::Vector3d eulerAng;
    double roll, pitch;
    int count = 0;
};