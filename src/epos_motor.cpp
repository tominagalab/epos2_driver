#include "epos_motor.hpp"

#include <Definitions.h>

#include <utility>

EposMotor::EposMotor(
  MotorConfig config, EposConnection & connection,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: config_(std::move(config)), connection_(connection), logger_(logger), clock_(std::move(clock))
{
}

EposMotor::~EposMotor()
{
  if (enabled_) {
    std_srvs::srv::SetBool::Response response;
    set_enabled(false, response);
    if (!response.success) {
      RCLCPP_ERROR(logger_, "%s: shutdown: %s", config_.name_space.c_str(), response.message.c_str());
    }
  }
}

bool EposMotor::initialize()
{
  auto * port = connection_.handle(config_.id);
  if (!port) {
    error_ = connection_.error();
  } else {
    unsigned int error = 0;
    if (VCS_SetDisableState(port, config_.id, &error)) {
      ready_ = true;
      enabled_ = false;
      error_.clear();
      RCLCPP_INFO(logger_, "%s (node %u) initialized and disabled",
        config_.name_space.c_str(), config_.id);
      return true;
    }
    error_ = api_error("VCS_SetDisableState", error);
  }
  RCLCPP_ERROR(logger_, "%s (node %u): %s",
    config_.name_space.c_str(), config_.id, error_.c_str());
  return false;
}

void EposMotor::log_feedback_error(const std::string & message)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= next_feedback_error_log_) {
    RCLCPP_ERROR(logger_, "%s: feedback: %s", config_.name_space.c_str(), message.c_str());
    next_feedback_error_log_ = now + std::chrono::seconds(5);
  }
}

bool EposMotor::read_feedback(MotorFeedback & feedback)
{
  auto * port = connection_.handle(config_.id);
  if (!port) {
    log_feedback_error(connection_.error());
    return false;
  }
  unsigned int error = 0;
  int current = 0;
  int velocity = 0;
  int position = 0;
  const char * failed = nullptr;
  if (!VCS_GetCurrentIsEx(port, config_.id, &current, &error)) {
    failed = "VCS_GetCurrentIsEx";
  } else if (!VCS_GetVelocityIs(port, config_.id, &velocity, &error)) {
    failed = "VCS_GetVelocityIs";
  } else if (!VCS_GetPositionIs(port, config_.id, &position, &error)) {
    failed = "VCS_GetPositionIs";
  }
  if (failed) {
    log_feedback_error(api_error(failed, error));
    return false;
  }
  feedback = {current, velocity, position};
  return true;
}

void EposMotor::command(int32_t value)
{
  if (!ready_ || !enabled_ || uncertain_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
      "%s: ignoring command while motor is not enabled", config_.name_space.c_str());
    return;
  }
  if (!config_.position_mode &&
    (static_cast<int64_t>(value) > config_.velocity.max_velocity ||
    static_cast<int64_t>(value) < -static_cast<int64_t>(config_.velocity.max_velocity)))
  {
    RCLCPP_ERROR(logger_, "%s: velocity command %d exceeds limit %u",
      config_.name_space.c_str(), value, config_.velocity.max_velocity);
    return;
  }
  unsigned int error = 0;
  const bool success = config_.position_mode ?
    VCS_MoveToPosition(connection_.handle(config_.id), config_.id, static_cast<long>(value), 1, 1, &error) :
    VCS_MoveWithVelocity(connection_.handle(config_.id), config_.id, static_cast<long>(value), &error);
  if (!success) {
    RCLCPP_ERROR(logger_, "%s: %s", config_.name_space.c_str(),
      api_error(config_.position_mode ? "VCS_MoveToPosition" : "VCS_MoveWithVelocity", error).c_str());
  }
}

void EposMotor::set_enabled(bool enable, std_srvs::srv::SetBool::Response & response)
{
  response.success = false;
  if (!ready_ && !initialize()) {
    response.message = error_;
    return;
  }
  auto * port = connection_.handle(config_.id);
  unsigned int error = 0;
  if (enable) {
    if (uncertain_) {
      response.message = "Motor state is uncertain; disable it before enabling";
      return;
    }
    if (enabled_) {
      response.success = true;
      response.message = "EPOS2 is already enabled";
      return;
    }
    const auto & profile = config_.position_mode ? config_.position : config_.velocity;
    const char * failed = nullptr;
    if (!VCS_ClearFault(port, config_.id, &error)) {
      failed = "VCS_ClearFault";
    } else if (!(config_.position_mode ?
      VCS_ActivateProfilePositionMode(port, config_.id, &error) :
      VCS_ActivateProfileVelocityMode(port, config_.id, &error)))
    {
      failed = config_.position_mode ?
        "VCS_ActivateProfilePositionMode" : "VCS_ActivateProfileVelocityMode";
    } else if (!VCS_SetMaxProfileVelocity(port, config_.id, profile.max_velocity, &error)) {
      failed = "VCS_SetMaxProfileVelocity";
    } else if (!(config_.position_mode ?
      VCS_SetPositionProfile(port, config_.id, profile.profile_velocity,
        profile.acceleration, profile.deceleration, &error) :
      VCS_SetVelocityProfile(port, config_.id, profile.acceleration, profile.deceleration, &error)))
    {
      failed = config_.position_mode ? "VCS_SetPositionProfile" : "VCS_SetVelocityProfile";
    } else {
      enabled_ = true;
      uncertain_ = true;
      if (!VCS_SetEnableState(port, config_.id, &error)) {
        failed = "VCS_SetEnableState";
      } else {
        uncertain_ = false;
      }
    }
    if (failed) {
      response.message = api_error(failed, error);
      if (enabled_) {
        unsigned int disable_error = 0;
        if (VCS_SetDisableState(port, config_.id, &disable_error)) {
          enabled_ = false;
          uncertain_ = false;
        } else {
          response.message += "; " + api_error("VCS_SetDisableState", disable_error);
        }
      }
      RCLCPP_ERROR(logger_, "%s: %s", config_.name_space.c_str(), response.message.c_str());
      return;
    }
    response.success = true;
    response.message = "EPOS2 enabled";
    return;
  }

  if (!enabled_) {
    response.success = true;
    response.message = "EPOS2 is already disabled";
    return;
  }
  const bool halted = config_.position_mode ?
    VCS_HaltPositionMovement(port, config_.id, &error) :
    VCS_HaltVelocityMovement(port, config_.id, &error);
  const auto halt_error = error;
  error = 0;
  const bool disabled = VCS_SetDisableState(port, config_.id, &error);
  if (disabled) {
    enabled_ = false;
    uncertain_ = false;
  } else {
    uncertain_ = true;
  }
  if (!halted || !disabled) {
    response.message = !halted ?
      api_error(config_.position_mode ? "VCS_HaltPositionMovement" : "VCS_HaltVelocityMovement",
        halt_error) : "";
    if (!disabled) {
      if (!response.message.empty()) {
        response.message += "; ";
      }
      response.message += api_error("VCS_SetDisableState", error);
    }
    RCLCPP_ERROR(logger_, "%s: %s", config_.name_space.c_str(), response.message.c_str());
    return;
  }
  response.success = true;
  response.message = "EPOS2 halted and disabled";
}
