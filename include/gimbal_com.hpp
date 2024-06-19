//
// Created by You, Jisang on 04/03/2024.
//

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gimbal.hpp"
#include <unistd.h>


class gimbal_com : public rclcpp::Node {
public:
    gimbal_com(Gimbal* gimbal) : Node("gimbal_com_node"), gimbal(gimbal) {
        // Subscribe to the input topic
        sub_ = create_subscription<geometry_msgs::msg::Vector3>(
                "/gimbal_command", 10, std::bind(&gimbal_com::inputCallback, this, std::placeholders::_1));

        // Advertise the output topic
        pub_ = create_publisher<geometry_msgs::msg::Vector3>("/gimbal_state", 10);
    }

private:
    //this function gets called whenever the node receives data from subcribed topic.
    void inputCallback(const geometry_msgs::msg::Vector3::SharedPtr input_msg) {
        // Array to send
        double dataToSend[3];
        dataToSend[0] = input_msg->x;
        dataToSend[1] = input_msg->y;
        dataToSend[2] = input_msg->z;

        //write command to MCU
        if (write(gimbal->getPortID(), &dataToSend, sizeof(dataToSend)) < 0) {
            std::cerr << "Error writing to gimbal" << std::endl;
            //RSFATAL("Error writing to gimbal");
        }
        //get MCU response
        double receivedData[3];
        if (read(gimbal->getPortID(), &receivedData, sizeof(receivedData)) < 0) {
            std::cerr << "Error reading from gimbal" << std::endl;
            //RSFATAL("Error reading from gimbal");
        }

        //Publish the real joint angles to the return topic
        auto output_msg = std::make_unique<geometry_msgs::msg::Vector3>();
        output_msg->x = receivedData[0];
        output_msg->y = receivedData[1];
        output_msg->z = receivedData[2];

        gimbal->setState(receivedData[0], receivedData[1], receivedData[2]);    //update the state in gimbal object

        // Publish the processed message to the output topic
        pub_->publish(std::move(output_msg));
    }

    Gimbal* gimbal;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
};