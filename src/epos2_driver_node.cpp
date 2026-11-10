#include <Definitions.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

class Epos2DriverNode : public rclcpp::Node
{
public:
  Epos2DriverNode()
  : Node("epos2_driver")
  {
    device_name_ = declare_parameter<std::string>("device_name", "EPOS2");
    protocol_stack_name_ =
      declare_parameter<std::string>("protocol_stack_name", "MAXON SERIAL V2");
    interface_name_ = declare_parameter<std::string>("interface_name", "USB");
    port_name_ = declare_parameter<std::string>("port_name", "USB0");
    node_id_ = declare_parameter<int>("node_id", 1);
    baudrate_ = declare_parameter<int>("baudrate", 1000000);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 500);
    profile_acceleration_ = declare_parameter<int>("profile_acceleration", 1000);
    profile_deceleration_ = declare_parameter<int>("profile_deceleration", 1000);

    if (node_id_ < 1 || node_id_ > 127) {
      throw std::invalid_argument("node_id must be between 1 and 127");
    }
    if (baudrate_ <= 0 || timeout_ms_ <= 0 ||
      profile_acceleration_ <= 0 || profile_deceleration_ <= 0)
    {
      throw std::invalid_argument(
              "baudrate, timeout_ms, profile_acceleration, and "
              "profile_deceleration must be positive");
    }

    unsigned int error_code = 0;
    device_handle_ = VCS_OpenDevice(
      device_name_.data(), protocol_stack_name_.data(), interface_name_.data(),
      port_name_.data(), &error_code);
    if (device_handle_ == nullptr) {
      throw std::runtime_error(api_error("VCS_OpenDevice", error_code));
    }

    if (!VCS_SetProtocolStackSettings(
        device_handle_, static_cast<unsigned int>(baudrate_),
        static_cast<unsigned int>(timeout_ms_), &error_code))
    {
      close_device();
      throw std::runtime_error(api_error("VCS_SetProtocolStackSettings", error_code));
    }

    if (!VCS_SetDisableState(device_handle_, node_id(), &error_code)) {
      close_device();
      throw std::runtime_error(api_error("VCS_SetDisableState", error_code));
    }

    velocity_subscription_ = create_subscription<std_msgs::msg::Int32>(
      "cmd_vel_rpm", 10,
      std::bind(&Epos2DriverNode::velocity_callback, this, std::placeholders::_1));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "enable",
      std::bind(
        &Epos2DriverNode::enable_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "Connected to EPOS2 on %s; motor remains disabled until enable=true",
      port_name_.c_str());
  }

  ~Epos2DriverNode() override
  {
    if (device_handle_ != nullptr) {
      if (enabled_) {
        unsigned int error_code = 0;
        if (!VCS_HaltVelocityMovement(device_handle_, node_id(), &error_code)) {
          RCLCPP_ERROR(get_logger(), "%s", api_error("VCS_HaltVelocityMovement", error_code).c_str());
        }
        error_code = 0;
        if (!VCS_SetDisableState(device_handle_, node_id(), &error_code)) {
          RCLCPP_ERROR(get_logger(), "%s", api_error("VCS_SetDisableState", error_code).c_str());
        }
      }
      close_device();
    }
  }

private:
  unsigned short node_id() const
  {
    return static_cast<unsigned short>(node_id_);
  }

  static std::string api_error(const char * function, unsigned int error_code)
  {
    return std::string(function) + " failed (error 0x" +
           [] (unsigned int code) {
             constexpr char digits[] = "0123456789ABCDEF";
             std::string result(8, '0');
             for (int index = 7; index >= 0; --index) {
               result[static_cast<std::size_t>(index)] = digits[code & 0xF];
               code >>= 4;
             }
             return result;
           }(error_code) + ")";
  }

  void close_device()
  {
    if (device_handle_ == nullptr) {
      return;
    }

    unsigned int error_code = 0;
    if (!VCS_CloseDevice(device_handle_, &error_code)) {
      RCLCPP_ERROR(get_logger(), "%s", api_error("VCS_CloseDevice", error_code).c_str());
    }
    device_handle_ = nullptr;
  }

  void velocity_callback(const std_msgs::msg::Int32::SharedPtr message)
  {
    if (!enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring velocity command while motor is disabled");
      return;
    }

    unsigned int error_code = 0;
    if (!VCS_MoveWithVelocity(
        device_handle_, node_id(), static_cast<long>(message->data), &error_code))
    {
      RCLCPP_ERROR(
        get_logger(), "%s", api_error("VCS_MoveWithVelocity", error_code).c_str());
    }
  }

  void enable_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    unsigned int error_code = 0;
    if (request->data) {
      if (enabled_) {
        response->success = true;
        response->message = "EPOS2 is already enabled";
        return;
      }
      if (!VCS_ClearFault(device_handle_, node_id(), &error_code)) {
        response->success = false;
        response->message = api_error("VCS_ClearFault", error_code);
        return;
      }
      if (!VCS_ActivateProfileVelocityMode(device_handle_, node_id(), &error_code)) {
        response->success = false;
        response->message = api_error("VCS_ActivateProfileVelocityMode", error_code);
        return;
      }
      if (!VCS_SetVelocityProfile(
          device_handle_, node_id(),
          static_cast<unsigned int>(profile_acceleration_),
          static_cast<unsigned int>(profile_deceleration_), &error_code))
      {
        response->success = false;
        response->message = api_error("VCS_SetVelocityProfile", error_code);
        return;
      }
      if (!VCS_SetEnableState(device_handle_, node_id(), &error_code)) {
        response->success = false;
        response->message = api_error("VCS_SetEnableState", error_code);
        return;
      }
      enabled_ = true;
      response->success = true;
      response->message = "EPOS2 enabled";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      return;
    }

    if (!enabled_) {
      response->success = true;
      response->message = "EPOS2 is already disabled";
      return;
    }

    const bool halted =
      VCS_HaltVelocityMovement(device_handle_, node_id(), &error_code) != 0;
    const unsigned int halt_error = error_code;
    error_code = 0;
    const bool disabled =
      VCS_SetDisableState(device_handle_, node_id(), &error_code) != 0;
    enabled_ = false;

    if (!halted || !disabled) {
      response->success = false;
      response->message = !halted ?
        api_error("VCS_HaltVelocityMovement", halt_error) :
        api_error("VCS_SetDisableState", error_code);
      return;
    }

    response->success = true;
    response->message = "EPOS2 halted and disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  std::string device_name_;
  std::string protocol_stack_name_;
  std::string interface_name_;
  std::string port_name_;
  int node_id_;
  int baudrate_;
  int timeout_ms_;
  int profile_acceleration_;
  int profile_deceleration_;
  void * device_handle_{nullptr};
  bool enabled_{false};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr velocity_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Epos2DriverNode>());
  } catch (const std::exception & exception) {
    std::cerr << "Failed to start EPOS2 driver: " << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
