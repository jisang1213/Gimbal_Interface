#include <string>
#include "rclcpp/rclcpp.hpp"
#include "gimbal_interface/gimbal_com.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    // get portname from config.YAML (e.g. /dev/ttyACM1)  -->  std::string portname = param_["gimbal"]("serial_port");
    std::string portname = "/dev/ttyACM0";
    auto com_node = std::make_shared<gimbal_com>(portname);

    executor.add_node(com_node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}