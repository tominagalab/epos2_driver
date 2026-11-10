#include "epos_driver_node.hpp"

#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include "rclcpp/expand_topic_or_service_name.hpp"

EposDriverNode::EposDriverNode()
: Node("epos_driver_node")
{
  ConnectionConfig connection_config;
  connection_config.device = declare_parameter<std::string>("device_name", "EPOS2");
  connection_config.protocol =
    declare_parameter<std::string>("protocol_stack_name", "MAXON SERIAL V2");
  connection_config.interface = declare_parameter<std::string>("interface_name", "USB");
  connection_config.port = declare_parameter<std::string>("port_name", "USB0");
  connection_config.baudrate = positive("usb_baudrate");
  connection_config.timeout = positive("timeout_ms");
  connection_config.can_baudrate = positive("can_baudrate");
  const auto gateway_id = declare_parameter<int64_t>("gateway_node_id", 4);
  if (gateway_id < 1 || gateway_id > 127) {
    throw std::invalid_argument("gateway_node_id must be between 1 and 127");
  }
  connection_config.gateway_id = static_cast<unsigned short>(gateway_id);

  const auto names = declare_parameter<std::vector<std::string>>("motor_names");
  if (names.empty()) {
    throw std::invalid_argument("motor_names must not be empty");
  }
  std::set<std::string> unique_names;
  std::set<std::string> namespaces;
  std::set<unsigned short> ids;
  std::vector<MotorConfig> configs;
  for (const auto & name : names) {
    if (name.empty() || !unique_names.insert(name).second) {
      throw std::invalid_argument("motor_names contains an empty or duplicate name");
    }
    const auto prefix = "motors." + name + ".";
    const auto id = declare_parameter<int64_t>(prefix + "node_id");
    if (id < 1 || id > 127 || !ids.insert(static_cast<unsigned short>(id)).second) {
      throw std::invalid_argument(prefix + "node_id must be unique and between 1 and 127");
    }
    MotorConfig config;
    config.id = static_cast<unsigned short>(id);
    config.name_space = declare_parameter<std::string>(prefix + "namespace");
    if (config.name_space.size() < 2 || config.name_space.front() != '/' ||
      config.name_space.back() == '/' || !namespaces.insert(config.name_space).second)
    {
      throw std::invalid_argument(prefix + "namespace must be unique and absolute without a trailing slash");
    }
    const auto mode = declare_parameter<std::string>(prefix + "mode");
    if (mode != "velocity" && mode != "position") {
      throw std::invalid_argument(prefix + "mode must be velocity or position");
    }
    config.position_mode = mode == "position";
    config.velocity = read_profile(prefix + "velocity.", false);
    config.position = read_profile(prefix + "position.", true);
    if (config.position.profile_velocity > config.position.max_velocity) {
      throw std::invalid_argument(prefix + "position.profile_velocity exceeds max_velocity");
    }
    rclcpp::expand_topic_or_service_name(
      config.name_space + (config.position_mode ? "/cmd_position_counts" : "/cmd_vel_rpm"),
      get_name(), get_namespace());
    rclcpp::expand_topic_or_service_name(
      config.name_space + "/enable", get_name(), get_namespace(), true);
    configs.push_back(std::move(config));
  }
  connection_config.use_can = ids.size() > ids.count(connection_config.gateway_id);
  connection_ = std::make_unique<EposConnection>(connection_config, get_logger());
  for (auto & config : configs) {
    auto motor = std::make_unique<EposMotor>(
      std::move(config), *connection_, get_logger(), get_clock());
    auto * ptr = motor.get();
    const auto & settings = motor->config();
    const auto topic = settings.name_space +
      (settings.position_mode ? "/cmd_position_counts" : "/cmd_vel_rpm");
    commands_.push_back(create_subscription<std_msgs::msg::Int32>(
      topic, 10, [ptr](const std_msgs::msg::Int32::SharedPtr msg) {ptr->command(msg->data);}));
    services_.push_back(create_service<std_srvs::srv::SetBool>(
      settings.name_space + "/enable",
      [ptr](const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        ptr->set_enabled(request->data, *response);
      }));
    motors_.push_back(std::move(motor));
  }
  for (const auto & motor : motors_) {
    motor->initialize();
  }
}

unsigned int EposDriverNode::positive(const std::string & name)
{
  const auto value = declare_parameter<int64_t>(name);
  if (value <= 0 ||
    static_cast<uint64_t>(value) > std::numeric_limits<unsigned int>::max())
  {
    throw std::invalid_argument(name + " must be a positive 32-bit unsigned integer");
  }
  return static_cast<unsigned int>(value);
}

Profile EposDriverNode::read_profile(const std::string & prefix, bool position)
{
  Profile profile{
    positive(prefix + "acceleration"), positive(prefix + "deceleration"),
    positive(prefix + "max_velocity"), 0};
  if (position) {
    profile.profile_velocity = positive(prefix + "profile_velocity");
  }
  return profile;
}
