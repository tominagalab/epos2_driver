#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class ActiveFingerPublisher : public rclcpp::Node
{
public:
  ActiveFingerPublisher()
  : Node("active_finger_publisher")
  {
    left_publisher_ = create_publisher<std_msgs::msg::Int32>(
      "/epos2/left_active_finger/cmd_vel_rpm", 10);
    right_publisher_ = create_publisher<std_msgs::msg::Int32>(
      "/epos2/right_active_finger/cmd_vel_rpm", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publish_commands();});
    RCLCPP_INFO(get_logger(), "Publishing 50 rpm to both active fingers at 10 Hz");
  }

private:
  void publish_commands()
  {
    std_msgs::msg::Int32 left_message;
    left_message.data = 50;
    left_publisher_->publish(left_message);

    std_msgs::msg::Int32 right_message;
    right_message.data = 50;
    right_publisher_->publish(right_message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActiveFingerPublisher>());
  rclcpp::shutdown();
  return 0;
}
