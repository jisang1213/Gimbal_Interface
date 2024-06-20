#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "gimbal_interface/gimbal.hpp"
#include "gimbal_interface/gimbal_com.hpp"
#include "gimbal_interface/commander.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    // For 
    // parameter::ParameterContainer & param_ = parameter::ParameterContainer::getRoot()["Raibot"];

    // get useGimbal from config.YAML  -->  bool usebool useGimbal = param_["gimbal"]("state");
    bool isReal = true, useGimbal = true;

    if(isReal && useGimbal) {
        // get portname from config.YAML (e.g. /dev/ttyACM1)  -->  std::string portname = param_["gimbal"]("serial_port");
        std::string portname = "/dev/ttyACM0";

        Gimbal gimbal(portname);

        auto com_node = std::make_shared<gimbal_com>(&gimbal);
        auto commander = std::make_shared<gimbal_commander>();

        //for active control mode with real robot, uncomment the line below and pass the world pointer.
        //commander->setWorld(&worldContainer); 

        executor.add_node(com_node);
        executor.add_node(commander);
        executor.spin();

        Eigen::MatrixXd H_matrix_left_cam = gimbal.getHmatrix_L();
        Eigen::MatrixXd H_matrix_right_cam = gimbal.getHmatrix_R();
        //now use these to transform the pointcloud from the camera frame to the gimbal base frame

        //for example, to transform the vector (3,2,5) from the left camera frame to the base frame,
        Eigen::VectorXd pos(4), transformed_pos(4);
        pos << 3,2,5, 1; //fourth element should always be 1
        transformed_pos = H_matrix_left_cam * pos;
 
        rclcpp::shutdown();
    }
    return 0;
}