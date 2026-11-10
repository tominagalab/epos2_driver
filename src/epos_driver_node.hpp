#pragma once

#include <memory>
#include <string>
#include <vector>

#include "epos_connection.hpp"
#include "epos_motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

class EposDriverNode : public rclcpp::Node
{
public:
  EposDriverNode();

private:
  unsigned int positive(const std::string & name);
  Profile read_profile(const std::string & prefix, bool position);

  std::unique_ptr<EposConnection> connection_;
  std::vector<std::unique_ptr<EposMotor>> motors_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> commands_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> services_;
};
