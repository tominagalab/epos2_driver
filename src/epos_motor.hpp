#pragma once

#include <cstdint>
#include <string>

#include "epos_connection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

struct Profile
{
  unsigned int acceleration;
  unsigned int deceleration;
  unsigned int max_velocity;
  unsigned int profile_velocity;
};

struct MotorConfig
{
  unsigned short id;
  std::string name_space;
  bool position_mode;
  Profile velocity;
  Profile position;
};

class EposMotor
{
public:
  EposMotor(MotorConfig config, EposConnection & connection,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  ~EposMotor();
  EposMotor(const EposMotor &) = delete;
  EposMotor & operator=(const EposMotor &) = delete;

  bool initialize();
  void command(int32_t value);
  void set_enabled(bool enable, std_srvs::srv::SetBool::Response & response);
  const MotorConfig & config() const {return config_;}

private:
  MotorConfig config_;
  EposConnection & connection_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  bool ready_{false};
  bool enabled_{false};
  bool uncertain_{false};
  std::string error_;
};
