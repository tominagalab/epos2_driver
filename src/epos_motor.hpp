#pragma once

#include <chrono>
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
  double gear_ratio;
  unsigned int encoder_pulses_per_revolution;
  Profile velocity;
  Profile position;
};

struct MotorFeedback
{
  int32_t current_ma;
  int32_t velocity_actual;
  int32_t position_counts;
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
  bool read_feedback(MotorFeedback & feedback);
  void set_enabled(bool enable, std_srvs::srv::SetBool::Response & response);
  const MotorConfig & config() const {return config_;}

private:
  void log_feedback_error(const std::string & message);
  MotorConfig config_;
  EposConnection & connection_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  bool ready_{false};
  bool enabled_{false};
  bool uncertain_{false};
  std::string error_;
  std::chrono::steady_clock::time_point next_feedback_error_log_{};
};
