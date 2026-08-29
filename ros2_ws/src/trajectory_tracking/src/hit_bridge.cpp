#include <cmath>
#include <memory>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sentry_msgs/msg/slaver_speed.hpp>
class HitBridge final : public rclcpp::Node {
 public:
  HitBridge() : Node("hit_bridge") {
    const auto commandTopic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto speedTopic = declare_parameter<std::string>("speed_topic", "/sentry_des_speed");
    const auto arrivedTopic = declare_parameter<std::string>("arrived_topic", "/dstar_status");
    commandPub_ = create_publisher<geometry_msgs::msg::Twist>(commandTopic, rclcpp::QoS(10).reliable());
    arrivedPub_ = create_publisher<std_msgs::msg::Bool>(arrivedTopic, rclcpp::QoS(10).reliable());
    speedSub_ = create_subscription<sentry_msgs::msg::SlaverSpeed>(
      speedTopic, rclcpp::QoS(10).reliable(),
      [this](sentry_msgs::msg::SlaverSpeed::ConstSharedPtr speed) {
        geometry_msgs::msg::Twist command;
        // Tracking already rotates map velocity into base_link exactly once.
        command.linear.x = speed->angle_target;
        command.linear.y = speed->angle_current;
        commandPub_->publish(command);
        std_msgs::msg::Bool arrived;
        arrived.data = std::fabs(command.linear.x) < 0.01 && std::fabs(command.linear.y) < 0.01;
        arrivedPub_->publish(arrived);
      });
  }
 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrivedPub_;
  rclcpp::Subscription<sentry_msgs::msg::SlaverSpeed>::SharedPtr speedSub_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HitBridge>());
  rclcpp::shutdown();
  return 0;
}
