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

    // get portname from config.YAML (e.g. /dev/ttyACM1)  -->  std::string portname = param_["gimbal"]("serial_port");
    std::string portname = "/dev/ttyACM0";
    auto com_node = std::make_shared<gimbal_com>(portname);
    auto commander = std::make_shared<gimbal_commander>();

    executor.add_node(com_node);
    executor.add_node(commander);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}