#include <gtest/gtest.h>
#include <cmath>
#include <hdl_localization/odometry_twist_contract.hpp>
TEST(OdometryTwistContract, rotates_world_velocity_and_covariance_into_base_link) {
  nav_msgs::msg::Odometry source;
  source.pose.pose.orientation.z = std::sqrt(0.5); source.pose.pose.orientation.w = std::sqrt(0.5);
  source.twist.twist.linear.x = 1.0; source.twist.twist.linear.y = 2.0;
  source.twist.covariance.fill(0.0); source.twist.covariance[0] = 4.0; source.twist.covariance[7] = 9.0; source.twist.covariance[14] = 16.0; source.twist.covariance[21] = 25.0;
  nav_msgs::msg::Odometry output;
  ASSERT_TRUE(hdl_localization::copyTwistInBaseLink(source, output));
  EXPECT_NEAR(output.twist.twist.linear.x, 2.0, 1e-12); EXPECT_NEAR(output.twist.twist.linear.y, -1.0, 1e-12);
  EXPECT_NEAR(output.twist.covariance[0], 9.0, 1e-12); EXPECT_NEAR(output.twist.covariance[7], 4.0, 1e-12); EXPECT_NEAR(output.twist.covariance[21], 0.0, 1e-12); EXPECT_NEAR(output.twist.covariance[28], 25.0, 1e-12);
}
TEST(OdometryTwistContract, zeros_and_marks_covariance_when_unavailable) {
  nav_msgs::msg::Odometry output; output.twist.twist.linear.x = 5.0;
  hdl_localization::markTwistUnavailable(output);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 0.0); EXPECT_DOUBLE_EQ(output.twist.covariance[0], 1.0e6); EXPECT_DOUBLE_EQ(output.twist.covariance[35], 1.0e6);
}
