#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "sentry_rviz_tools/goal3d_tool.hpp"

namespace sentry_rviz_tools {
Goal3DTool::Goal3DTool() {
  shortcut_key_ = 'g';
  topic_property_ = new rviz_common::properties::StringProperty(
      "Topic", "/goal", "PoseStamped topic consumed by the ROS2 planner manual-goal input.",
      getPropertyContainer(), SLOT(updateTopic()), this);
}

void Goal3DTool::onInitialize() {
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  updateTopic();
}

void Goal3DTool::updateTopic() {
  if (node_) publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_property_->getStdString(), rclcpp::QoS(1));
}

geometry_msgs::msg::PoseStamped Goal3DTool::makeGoal(
    double x, double y, double z, double yaw, const std::string& frame_id) const {
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame_id;
  if (node_) goal.header.stamp = node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.z = std::sin(yaw * 0.5);
  goal.pose.orientation.w = std::cos(yaw * 0.5);
  return goal;
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta) {
  if (publisher_) publisher_->publish(makeGoal(x, y, z, theta, context_->getFixedFrame().toStdString()));
}
}  // namespace sentry_rviz_tools

PLUGINLIB_EXPORT_CLASS(sentry_rviz_tools::Goal3DTool, rviz_common::Tool)
