#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace {

class RclcppGuard {
public:
  RclcppGuard() { rclcpp::init(0, nullptr); }
  ~RclcppGuard() { rclcpp::shutdown(); }
};

TEST(EngineDefaults, BareNodeUsesFpfhRansacFallback) {
  RclcppGuard guard;
  const auto node = std::make_shared<rclcpp::Node>("bare_default_test");
  EXPECT_EQ(node->declare_parameter<std::string>("global_localization_engine", "FPFH_RANSAC"),
            "FPFH_RANSAC");
}

TEST(EngineDefaults, StandardLaunchConfigurationSelectsBbs) {
  RclcppGuard guard;
  rclcpp::NodeOptions options;
  options.append_parameter_override("global_localization_engine", "BBS");
  const auto node = std::make_shared<rclcpp::Node>("launch_default_test", options);
  EXPECT_EQ(node->declare_parameter<std::string>("global_localization_engine", "FPFH_RANSAC"),
            "BBS");
}

TEST(EngineDefaults, TeaserIsExplicitlyUnavailableInProductionBuild) {
  RclcppGuard guard;
  rclcpp::NodeOptions options;
  options.append_parameter_override("global_localization_engine", "FPFH_TEASER");
  const auto node = std::make_shared<rclcpp::Node>("teaser_unavailable_test", options);
  EXPECT_EQ(node->declare_parameter<std::string>("global_localization_engine", "FPFH_RANSAC"),
            "FPFH_TEASER");
}

}  // namespace
