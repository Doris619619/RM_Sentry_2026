#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
namespace hdl_localization {
inline void markTwistUnavailable(nav_msgs::msg::Odometry& output) {
  output.twist.twist = geometry_msgs::msg::Twist();
  output.twist.covariance.fill(0.0);
  for (int i = 0; i < 6; ++i) output.twist.covariance[6 * i + i] = 1.0e6;
}
inline bool copyTwistInBaseLink(const nav_msgs::msg::Odometry& source, nav_msgs::msg::Odometry& output) {
  const auto& q = source.pose.pose.orientation;
  Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!std::isfinite(orientation.norm()) || orientation.norm() < 1.0e-12) return false;
  orientation.normalize();
  const Eigen::Matrix3d bodyFromOdom = orientation.toRotationMatrix().transpose();
  const auto& world = source.twist.twist.linear;
  const Eigen::Vector3d body = bodyFromOdom * Eigen::Vector3d(world.x, world.y, world.z);
  output.twist.twist.linear.x = body.x(); output.twist.twist.linear.y = body.y(); output.twist.twist.linear.z = body.z();
  output.twist.twist.angular = source.twist.twist.angular;
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance;
  std::copy(source.twist.covariance.begin(), source.twist.covariance.end(), covariance.data());
  Eigen::Matrix<double, 6, 6> rotation = Eigen::Matrix<double, 6, 6>::Zero();
  rotation.topLeftCorner<3, 3>() = bodyFromOdom; rotation.bottomRightCorner<3, 3>() = bodyFromOdom;
  const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> rotated = rotation * covariance * rotation.transpose();
  std::copy(rotated.data(), rotated.data() + rotated.size(), output.twist.covariance.begin());
  return true;
}
}  // namespace hdl_localization
