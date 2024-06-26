//
// Created by You, Jisang on 04/03/2024.
//
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include <unistd.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "gimbal.hpp"

///***check message headers***///

class gimbal_com : public rclcpp::Node {
public:
    gimbal_com(std::string portname) : Node("gimbal_com_node"), gimbal(portname) {
        // Subscribe to the input topic
        sub = create_subscription<geometry_msgs::msg::Vector3>(
                "railab_raibo/gimbal_command", 10, std::bind(&gimbal_com::inputCallback, this, std::placeholders::_1));
        jointstate = create_publisher<geometry_msgs::msg::Vector3>("joint_state", 10);

        // Advertise the output topic
        pubL = create_publisher<geometry_msgs::msg::Transform>("railab_raibo/L_cam", 10);
        pubR = create_publisher<geometry_msgs::msg::Transform>("railab_raibo/R_cam", 10);
    }

private:
    //this function gets called whenever the node receives data from subcribed topic.
    void inputCallback(const geometry_msgs::msg::Vector3::SharedPtr command) {
        // Array to send
        double dataToSend[3];
        dataToSend[0] = command->x;
        dataToSend[1] = command->y;
        dataToSend[2] = command->z;

        //write command to MCU
        if (write(gimbal.getPortID(), &dataToSend, sizeof(dataToSend)) < 0) {
            std::cerr << "Error writing to gimbal" << std::endl;
            //RSFATAL("Error writing to gimbal");
        }
        //get MCU response
        double receivedData[3];
        if (read(gimbal.getPortID(), &receivedData, sizeof(receivedData)) < 0) {
            std::cerr << "Error reading from gimbal" << std::endl;
            //RSFATAL("Error reading from gimbal");
        }

        auto state = geometry_msgs::msg::Vector3();
        state.x = receivedData[0];
        state.y = receivedData[1];
        state.z = receivedData[2];
        jointstate->publish(state);

        //update the state in gimbal
        gimbal.setState(receivedData[0], receivedData[1], receivedData[2]);    //update the state in gimbal object

        rotL = gimbal.getRot_L();
        posL = gimbal.getPos_L();

        rotR = gimbal.getRot_R();
        posR = gimbal.getPos_R();

        // Convert the rotation matrix to a quaternion
        Eigen::Quaterniond quatL(rotL);
        Eigen::Quaterniond quatR(rotR);

        //Publish the transformation back to raisin
        auto transform_L = std::make_unique<geometry_msgs::msg::Transform>();
        auto transform_R = std::make_unique<geometry_msgs::msg::Transform>();

        //Left cam
        transform_L->translation.x = posL.x();
        transform_L->translation.y = posL.y();
        transform_L->translation.z = posL.z();
        
        transform_L->rotation.w = quatL.w();
        transform_L->rotation.x = quatL.x();
        transform_L->rotation.y = quatL.y();
        transform_L->rotation.z = quatL.z();
        
        //Right cam
        transform_R->translation.x = posR.x();
        transform_R->translation.y = posR.y();
        transform_R->translation.z = posR.z();
        
        transform_R->rotation.w = quatR.w();
        transform_R->rotation.x = quatR.x();
        transform_R->rotation.y = quatR.y();
        transform_R->rotation.z = quatR.z();

        // Publish the processed message to the output topic
        pubL->publish(std::move(transform_L));
        pubR->publish(std::move(transform_R));
    }

    Gimbal gimbal;
    Eigen::Matrix3d rotL, rotR;
    Eigen::Vector3d posL, posR;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr jointstate;
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr pubL, pubR;
};