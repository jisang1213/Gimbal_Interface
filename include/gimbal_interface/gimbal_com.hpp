//
// Created by You, Jisang on 04/03/2024.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <unistd.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gimbal_interface/msg/double_array12.hpp"
#include "gimbal.hpp"


class gimbal_com : public rclcpp::Node {
public:
    gimbal_com(std::string portname) : Node("gimbal_com_node"), gimbal(portname) {
        // Subscribe to the input topic
        sub_ = create_subscription<geometry_msgs::msg::Vector3>(
                "/gimbal_command", 10, std::bind(&gimbal_com::inputCallback, this, std::placeholders::_1));

        // Advertise the output topic
        pub_ = create_publisher<gimbal_interface::msg::DoubleArray12>("railab_raibo/gimbal_state", 10);
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

        //update the state in gimbal
        gimbal.setState(receivedData[0], receivedData[1], receivedData[2]);    //update the state in gimbal object

        Eigen::MatrixXd H_matrix_left_cam = gimbal.getHmatrix_L();
        Eigen::MatrixXd H_matrix_right_cam = gimbal.getHmatrix_R();

        //Publish the transformation back to raisin
        auto output_msg = std::make_unique<gimbal_interface::msg::DoubleArray12>();

        //rotation
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                output_msg->transform_L[3*i + j] = H_matrix_left_cam(i,j);
                output_msg->transform_R[3*i + j] = H_matrix_right_cam(i,j);
            }
        }

        //position
        for(int i=0; i<3; i++){
            output_msg->transform_L[9+i] = H_matrix_left_cam(i,3);
            output_msg->transform_R[9+i] = H_matrix_right_cam(i,3);
        }

        // Publish the processed message to the output topic
        pub_->publish(std::move(output_msg));
    }

    Gimbal gimbal;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
    rclcpp::Publisher<gimbal_interface::msg::DoubleArray12>::SharedPtr pub_;
};