#include <cstdio>
#include <memory>

#include "epos2_driver/msg/motor_feedback.hpp"
#include "rclcpp/rclcpp.hpp"

class ActiveFingerSubscriber : public rclcpp::Node
{
public:
  ActiveFingerSubscriber()
  : Node("active_finger_subscriber")
  {
    left_subscription_ =
      create_subscription<epos2_driver::msg::MotorFeedback>(
      "/epos2/left_active_finger/feedback", 10,
      [this](epos2_driver::msg::MotorFeedback::ConstSharedPtr message) {
        print_feedback("left", *message);
      });
    right_subscription_ =
      create_subscription<epos2_driver::msg::MotorFeedback>(
      "/epos2/right_active_finger/feedback", 10,
      [this](epos2_driver::msg::MotorFeedback::ConstSharedPtr message) {
        print_feedback("right", *message);
      });
  }

private:
  static void print_feedback(
    const char * side, const epos2_driver::msg::MotorFeedback & message)
  {
    std::printf(
      "[%s] stamp=%d.%09u current=%d mA velocity=%d position=%d "
      "output_velocity=%.3f output_position=%.3f rad\n",
      side,
      static_cast<int>(message.header.stamp.sec),
      static_cast<unsigned int>(message.header.stamp.nanosec),
      static_cast<int>(message.current_ma),
      static_cast<int>(message.velocity_actual),
      static_cast<int>(message.position_counts),
      message.output_shaft_velocity,
      message.output_shaft_position_rad);
    std::fflush(stdout);
  }

  rclcpp::Subscription<epos2_driver::msg::MotorFeedback>::SharedPtr left_subscription_;
  rclcpp::Subscription<epos2_driver::msg::MotorFeedback>::SharedPtr right_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActiveFingerSubscriber>());
  rclcpp::shutdown();
  return 0;
}
