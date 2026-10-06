#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/chassis_feedback.hpp"

class WheelVelocityNode : public rclcpp::Node
{
public:
  WheelVelocityNode()
  : Node("wheel_velocity_node")
  {
    const std::string input_topic =
      declare_parameter<std::string>("input_topic", "chassis_feedback");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "wheel_velocity");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link_hf");

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, qos);
    subscription_ =
      create_subscription<robot_interfaces::msg::ChassisFeedback>(
      input_topic, qos,
      std::bind(
        &WheelVelocityNode::feedbackCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "wheel velocity bridge: %s -> %s, frame=%s",
      input_topic.c_str(), output_topic.c_str(), frame_id_.c_str());
  }

private:
  void feedbackCallback(
    const robot_interfaces::msg::ChassisFeedback::SharedPtr msg)
  {
    if (!std::isfinite(msg->vx) || !std::isfinite(msg->vy) ||
      !std::isfinite(msg->vw))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "discarding non-finite chassis feedback");
      return;
    }

    geometry_msgs::msg::TwistStamped output;
    output.header = msg->header;
    if (!frame_id_.empty()) {
      output.header.frame_id = frame_id_;
    }
    output.twist.linear.x = msg->vx;
    output.twist.linear.y = msg->vy;
    output.twist.angular.z = msg->vw;
    publisher_->publish(output);
  }

  std::string frame_id_;
  rclcpp::Subscription<robot_interfaces::msg::ChassisFeedback>::SharedPtr
    subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocityNode>());
  rclcpp::shutdown();
  return 0;
}
