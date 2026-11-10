#include "epos_driver_node.hpp"

#include <iostream>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EposDriverNode>());
  } catch (const std::exception & exception) {
    std::cerr << "Failed to start EPOS driver: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
