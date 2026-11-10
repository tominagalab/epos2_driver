#include "epos_connection.hpp"

#include <Definitions.h>

#include <iomanip>
#include <sstream>

std::string api_error(const char * function, unsigned int code)
{
  std::ostringstream stream;
  stream << function << " failed (error 0x" << std::uppercase << std::hex <<
    std::setw(8) << std::setfill('0') << code << ")";
  return stream.str();
}

EposConnection::EposConnection(const ConnectionConfig & config, rclcpp::Logger logger)
: logger_(logger), gateway_id_(config.gateway_id)
{
  auto device = config.device;
  auto protocol = config.protocol;
  auto interface = config.interface;
  auto port = config.port;
  unsigned int error = 0;
  device_ = VCS_OpenDevice(
    device.data(), protocol.data(), interface.data(), port.data(), &error);
  if (!device_) {
    gateway_error_ = api_error("VCS_OpenDevice", error);
    RCLCPP_ERROR(logger_, "%s", gateway_error_.c_str());
    return;
  }
  if (!VCS_SetProtocolStackSettings(device_, config.baudrate, config.timeout, &error)) {
    gateway_error_ = api_error("VCS_SetProtocolStackSettings", error);
    RCLCPP_ERROR(logger_, "%s", gateway_error_.c_str());
    close_device();
    return;
  }
  if (config.use_can) {
    std::string can_protocol = "CANopen";
    subdevice_ = VCS_OpenSubDevice(device_, device.data(), can_protocol.data(), &error);
    if (!subdevice_) {
      can_error_ = api_error("VCS_OpenSubDevice", error);
    } else {
      unsigned int actual_baudrate = 0;
      if (!VCS_GetGatewaySettings(subdevice_, &actual_baudrate, &error)) {
        can_error_ = api_error("VCS_GetGatewaySettings", error);
      } else if (actual_baudrate != config.can_baudrate) {
        can_error_ = "CAN baudrate mismatch: expected " + std::to_string(config.can_baudrate) +
          ", gateway reports " + std::to_string(actual_baudrate);
      }
    }
  }
  if (!can_error_.empty()) {
    RCLCPP_ERROR(logger_, "%s", can_error_.c_str());
  }
}

EposConnection::~EposConnection()
{
  unsigned int error = 0;
  if (subdevice_ && !VCS_CloseSubDevice(subdevice_, &error)) {
    RCLCPP_ERROR(logger_, "%s", api_error("VCS_CloseSubDevice", error).c_str());
  }
  close_device();
}

void EposConnection::close_device()
{
  if (device_) {
    unsigned int error = 0;
    if (!VCS_CloseDevice(device_, &error)) {
      RCLCPP_ERROR(logger_, "%s", api_error("VCS_CloseDevice", error).c_str());
    }
    device_ = nullptr;
  }
}

void * EposConnection::handle(unsigned short node_id) const
{
  if (!device_ || !gateway_error_.empty()) {
    return nullptr;
  }
  return node_id == gateway_id_ ? device_ : (can_error_.empty() ? subdevice_ : nullptr);
}

const std::string & EposConnection::error() const
{
  return !gateway_error_.empty() ? gateway_error_ : can_error_;
}
