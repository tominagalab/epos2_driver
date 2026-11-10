#include <Definitions.h>

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

class EposDriverNode : public rclcpp::Node
{
  using Int32 = std_msgs::msg::Int32;
  using SetBool = std_srvs::srv::SetBool;

  struct Profile
  {
    unsigned int acceleration;
    unsigned int deceleration;
    unsigned int max_velocity;
    unsigned int profile_velocity;
  };

  struct Motor
  {
    unsigned short id;
    std::string name_space;
    bool position_mode;
    Profile velocity;
    Profile position;
    bool ready{false};
    bool enabled{false};
    bool uncertain{false};
    std::string error;
    rclcpp::Subscription<Int32>::SharedPtr command;
    rclcpp::Service<SetBool>::SharedPtr enable;
  };

public:
  EposDriverNode()
  : Node("epos_driver_node")
  {
    auto device = declare_parameter<std::string>("device_name", "EPOS2");
    auto protocol = declare_parameter<std::string>("protocol_stack_name", "MAXON SERIAL V2");
    auto interface = declare_parameter<std::string>("interface_name", "USB");
    auto port = declare_parameter<std::string>("port_name", "USB0");
    const auto baudrate = positive("usb_baudrate");
    const auto timeout = positive("timeout_ms");
    const auto can_baudrate = positive("can_baudrate");
    const auto gateway_id = declare_parameter<int64_t>("gateway_node_id", 4);
    if (gateway_id < 1 || gateway_id > 127) {
      throw std::invalid_argument("gateway_node_id must be between 1 and 127");
    }
    gateway_id_ = static_cast<unsigned short>(gateway_id);

    const auto names = declare_parameter<std::vector<std::string>>("motor_names");
    if (names.empty()) {
      throw std::invalid_argument("motor_names must not be empty");
    }
    std::set<std::string> unique_names;
    std::set<std::string> namespaces;
    std::set<unsigned short> ids;
    for (const auto & name : names) {
      if (name.empty() || !unique_names.insert(name).second) {
        throw std::invalid_argument("motor_names contains an empty or duplicate name");
      }
      const auto prefix = "motors." + name + ".";
      const auto id = declare_parameter<int64_t>(prefix + "node_id");
      if (id < 1 || id > 127 || !ids.insert(static_cast<unsigned short>(id)).second) {
        throw std::invalid_argument(prefix + "node_id must be unique and between 1 and 127");
      }
      auto motor = std::make_unique<Motor>();
      motor->id = static_cast<unsigned short>(id);
      motor->name_space = declare_parameter<std::string>(prefix + "namespace");
      if (motor->name_space.size() < 2 || motor->name_space.front() != '/' ||
        motor->name_space.back() == '/' || !namespaces.insert(motor->name_space).second)
      {
        throw std::invalid_argument(prefix + "namespace must be unique and absolute without a trailing slash");
      }
      const auto mode = declare_parameter<std::string>(prefix + "mode");
      if (mode != "velocity" && mode != "position") {
        throw std::invalid_argument(prefix + "mode must be velocity or position");
      }
      motor->position_mode = mode == "position";
      motor->velocity = read_profile(prefix + "velocity.", false);
      motor->position = read_profile(prefix + "position.", true);
      if (motor->position.profile_velocity > motor->position.max_velocity) {
        throw std::invalid_argument(prefix + "position.profile_velocity exceeds max_velocity");
      }
      motors_.push_back(std::move(motor));
    }
    for (const auto & motor : motors_) {
      auto * ptr = motor.get();
      const auto topic = motor->name_space +
        (motor->position_mode ? "/cmd_position_counts" : "/cmd_vel_rpm");
      motor->command = create_subscription<Int32>(
        topic, 10, [this, ptr](const Int32::SharedPtr msg) { command(*ptr, msg->data); });
      motor->enable = create_service<SetBool>(
        motor->name_space + "/enable",
        [this, ptr](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
          set_enabled(*ptr, request->data, *response);
        });
    }

    unsigned int error = 0;
    device_ = VCS_OpenDevice(
      device.data(), protocol.data(), interface.data(), port.data(), &error);
    if (!device_) {
      gateway_error_ = api_error("VCS_OpenDevice", error);
      RCLCPP_ERROR(get_logger(), "%s", gateway_error_.c_str());
      return;
    }
    if (!VCS_SetProtocolStackSettings(device_, baudrate, timeout, &error)) {
      gateway_error_ = api_error("VCS_SetProtocolStackSettings", error);
      RCLCPP_ERROR(get_logger(), "%s", gateway_error_.c_str());
      close_device();
      return;
    }
    if (ids.size() > ids.count(gateway_id_)) {
      std::string can_protocol = "CANopen";
      subdevice_ = VCS_OpenSubDevice(device_, device.data(), can_protocol.data(), &error);
      if (!subdevice_) {
        can_error_ = api_error("VCS_OpenSubDevice", error);
      } else {
        unsigned int actual_baudrate = 0;
        if (!VCS_GetGatewaySettings(subdevice_, &actual_baudrate, &error)) {
          can_error_ = api_error("VCS_GetGatewaySettings", error);
        } else if (actual_baudrate != can_baudrate) {
          can_error_ = "CAN baudrate mismatch: expected " + std::to_string(can_baudrate) +
            ", gateway reports " + std::to_string(actual_baudrate);
        }
      }
    }
    if (!can_error_.empty()) {
      RCLCPP_ERROR(get_logger(), "%s", can_error_.c_str());
    }
    for (const auto & motor : motors_) {
      initialize(*motor);
    }
  }

  ~EposDriverNode() override
  {
    for (const auto & motor : motors_) {
      if (motor->enabled) {
        SetBool::Response response;
        set_enabled(*motor, false, response);
        if (!response.success) {
          RCLCPP_ERROR(get_logger(), "%s: shutdown: %s",
            motor->name_space.c_str(), response.message.c_str());
        }
      }
    }
    unsigned int error = 0;
    if (subdevice_ && !VCS_CloseSubDevice(subdevice_, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", api_error("VCS_CloseSubDevice", error).c_str());
    }
    close_device();
  }

private:
  unsigned int positive(const std::string & name)
  {
    const auto value = declare_parameter<int64_t>(name);
    if (value <= 0 ||
      static_cast<uint64_t>(value) > std::numeric_limits<unsigned int>::max())
    {
      throw std::invalid_argument(name + " must be a positive 32-bit unsigned integer");
    }
    return static_cast<unsigned int>(value);
  }

  Profile read_profile(const std::string & prefix, bool position)
  {
    Profile profile{
      positive(prefix + "acceleration"), positive(prefix + "deceleration"),
      positive(prefix + "max_velocity"), 0};
    if (position) {
      profile.profile_velocity = positive(prefix + "profile_velocity");
    }
    return profile;
  }

  static std::string api_error(const char * function, unsigned int code)
  {
    std::ostringstream stream;
    stream << function << " failed (error 0x" << std::uppercase << std::hex <<
      std::setw(8) << std::setfill('0') << code << ")";
    return stream.str();
  }

  void close_device()
  {
    if (device_) {
      unsigned int error = 0;
      if (!VCS_CloseDevice(device_, &error)) {
        RCLCPP_ERROR(get_logger(), "%s", api_error("VCS_CloseDevice", error).c_str());
      }
      device_ = nullptr;
    }
  }

  void * handle(const Motor & motor) const
  {
    if (!device_ || !gateway_error_.empty()) {
      return nullptr;
    }
    return motor.id == gateway_id_ ? device_ : (can_error_.empty() ? subdevice_ : nullptr);
  }

  bool initialize(Motor & motor)
  {
    auto * port = handle(motor);
    if (!port) {
      motor.error = !gateway_error_.empty() ? gateway_error_ : can_error_;
    } else {
      unsigned int error = 0;
      if (VCS_SetDisableState(port, motor.id, &error)) {
        motor.ready = true;
        motor.enabled = false;
        motor.error.clear();
        RCLCPP_INFO(get_logger(), "%s (node %u) initialized and disabled",
          motor.name_space.c_str(), motor.id);
        return true;
      }
      motor.error = api_error("VCS_SetDisableState", error);
    }
    RCLCPP_ERROR(get_logger(), "%s (node %u): %s",
      motor.name_space.c_str(), motor.id, motor.error.c_str());
    return false;
  }

  void command(Motor & motor, int32_t value)
  {
    if (!motor.ready || !motor.enabled || motor.uncertain) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "%s: ignoring command while motor is not enabled", motor.name_space.c_str());
      return;
    }
    if (!motor.position_mode &&
      (static_cast<int64_t>(value) > motor.velocity.max_velocity ||
      static_cast<int64_t>(value) < -static_cast<int64_t>(motor.velocity.max_velocity)))
    {
      RCLCPP_ERROR(get_logger(), "%s: velocity command %d exceeds limit %u",
        motor.name_space.c_str(), value, motor.velocity.max_velocity);
      return;
    }
    unsigned int error = 0;
    const bool success = motor.position_mode ?
      VCS_MoveToPosition(handle(motor), motor.id, static_cast<long>(value), 1, 1, &error) :
      VCS_MoveWithVelocity(handle(motor), motor.id, static_cast<long>(value), &error);
    if (!success) {
      RCLCPP_ERROR(get_logger(), "%s: %s", motor.name_space.c_str(),
        api_error(motor.position_mode ? "VCS_MoveToPosition" : "VCS_MoveWithVelocity", error).c_str());
    }
  }

  void set_enabled(Motor & motor, bool enable, SetBool::Response & response)
  {
    response.success = false;
    if (!motor.ready && !initialize(motor)) {
      response.message = motor.error;
      return;
    }
    auto * port = handle(motor);
    unsigned int error = 0;
    if (enable) {
      if (motor.uncertain) {
        response.message = "Motor state is uncertain; disable it before enabling";
        return;
      }
      if (motor.enabled) {
        response.success = true;
        response.message = "EPOS2 is already enabled";
        return;
      }
      const auto & profile = motor.position_mode ? motor.position : motor.velocity;
      const char * failed = nullptr;
      if (!VCS_ClearFault(port, motor.id, &error)) {
        failed = "VCS_ClearFault";
      } else if (!(motor.position_mode ?
        VCS_ActivateProfilePositionMode(port, motor.id, &error) :
        VCS_ActivateProfileVelocityMode(port, motor.id, &error)))
      {
        failed = motor.position_mode ?
          "VCS_ActivateProfilePositionMode" : "VCS_ActivateProfileVelocityMode";
      } else if (!VCS_SetMaxProfileVelocity(port, motor.id, profile.max_velocity, &error)) {
        failed = "VCS_SetMaxProfileVelocity";
      } else if (!(motor.position_mode ?
        VCS_SetPositionProfile(port, motor.id, profile.profile_velocity,
          profile.acceleration, profile.deceleration, &error) :
        VCS_SetVelocityProfile(port, motor.id, profile.acceleration, profile.deceleration, &error)))
      {
        failed = motor.position_mode ? "VCS_SetPositionProfile" : "VCS_SetVelocityProfile";
      } else {
        motor.enabled = true;
        motor.uncertain = true;
        if (!VCS_SetEnableState(port, motor.id, &error)) {
          failed = "VCS_SetEnableState";
        } else {
          motor.uncertain = false;
        }
      }
      if (failed) {
        response.message = api_error(failed, error);
        if (motor.enabled) {
          unsigned int disable_error = 0;
          if (VCS_SetDisableState(port, motor.id, &disable_error)) {
            motor.enabled = false;
            motor.uncertain = false;
          } else {
            response.message += "; " + api_error("VCS_SetDisableState", disable_error);
          }
        }
        RCLCPP_ERROR(get_logger(), "%s: %s", motor.name_space.c_str(), response.message.c_str());
        return;
      }
      response.success = true;
      response.message = "EPOS2 enabled";
      return;
    }

    if (!motor.enabled) {
      response.success = true;
      response.message = "EPOS2 is already disabled";
      return;
    }
    const bool halted = motor.position_mode ?
      VCS_HaltPositionMovement(port, motor.id, &error) :
      VCS_HaltVelocityMovement(port, motor.id, &error);
    const auto halt_error = error;
    error = 0;
    const bool disabled = VCS_SetDisableState(port, motor.id, &error);
    if (disabled) {
      motor.enabled = false;
      motor.uncertain = false;
    } else {
      motor.uncertain = true;
    }
    if (!halted || !disabled) {
      response.message = !halted ?
        api_error(motor.position_mode ? "VCS_HaltPositionMovement" : "VCS_HaltVelocityMovement",
          halt_error) : "";
      if (!disabled) {
        if (!response.message.empty()) {
          response.message += "; ";
        }
        response.message += api_error("VCS_SetDisableState", error);
      }
      RCLCPP_ERROR(get_logger(), "%s: %s", motor.name_space.c_str(), response.message.c_str());
      return;
    }
    response.success = true;
    response.message = "EPOS2 halted and disabled";
  }

  unsigned short gateway_id_{0};
  void * device_{nullptr};
  void * subdevice_{nullptr};
  std::string gateway_error_;
  std::string can_error_;
  std::vector<std::unique_ptr<Motor>> motors_;
};

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
