#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

#include "sentry_rviz_tools/goal3d_tool.hpp"

TEST(Goal3DTool, PluginlibDiscoversGoalTool) {
  pluginlib::ClassLoader<rviz_common::Tool> loader(
      "rviz_common", "rviz_common::Tool");
  EXPECT_TRUE(loader.isClassAvailable("sentry_rviz_tools/Goal3DTool"));
}

TEST(Goal3DTool, BuildsMapFramePoseStampedGoal) {
  auto tool = std::make_unique<sentry_rviz_tools::Goal3DTool>();
  const auto goal = tool->makeGoal(1.25, -2.5, 0.75, M_PI / 2.0, "map");
  EXPECT_EQ(goal.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(goal.pose.position.x, 1.25);
  EXPECT_DOUBLE_EQ(goal.pose.position.y, -2.5);
  EXPECT_DOUBLE_EQ(goal.pose.position.z, 0.75);
  EXPECT_NEAR(goal.pose.orientation.z, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(goal.pose.orientation.w, std::sqrt(0.5), 1e-6);
}
