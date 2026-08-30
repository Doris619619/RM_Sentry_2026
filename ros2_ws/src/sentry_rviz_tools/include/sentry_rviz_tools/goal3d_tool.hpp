#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "sentry_rviz_tools/pose3d_tool.hpp"

namespace sentry_rviz_tools {
class Goal3DTool final : public Pose3DTool {
  Q_OBJECT
public:
  Goal3DTool();
  void onInitialize() override;
  geometry_msgs::msg::PoseStamped makeGoal(double x, double y, double z, double yaw,
                                           const std::string& frame_id) const;
protected:
  void onPoseSet(double x, double y, double z, double theta) override;
private Q_SLOTS:
  void updateTopic();
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rviz_common::properties::StringProperty* topic_property_{nullptr};
};
}  // namespace sentry_rviz_tools
