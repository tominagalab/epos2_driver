#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

struct ConnectionConfig
{
  std::string device;
  std::string protocol;
  std::string interface;
  std::string port;
  unsigned int baudrate;
  unsigned int timeout;
  unsigned int can_baudrate;
  unsigned short gateway_id;
  bool use_can;
};

std::string api_error(const char * function, unsigned int code);

class EposConnection
{
public:
  EposConnection(const ConnectionConfig & config, rclcpp::Logger logger);
  ~EposConnection();
  EposConnection(const EposConnection &) = delete;
  EposConnection & operator=(const EposConnection &) = delete;

  void * handle(unsigned short node_id) const;
  const std::string & error() const;

private:
  void close_device();
  rclcpp::Logger logger_;
  unsigned short gateway_id_;
  void * device_{nullptr};
  void * subdevice_{nullptr};
  std::string gateway_error_;
  std::string can_error_;
};
