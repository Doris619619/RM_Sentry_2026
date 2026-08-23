// 此文件用于单元验证 HDL 定位链中 map 到 odom 的 TF2 计算契约。
#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <hdl_localization/tf_contract.hpp>

namespace {

// 此函数用于生成绕 Z 轴旋转的刚体变换；输入为平移和偏航角，输出为 Eigen 等距变换，不产生 ROS 通信副作用。
Eigen::Isometry3d make_transform(
  const Eigen::Vector3d& translation,
  const double yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

// 此测试用于验证 T_map_odom 右乘 T_odom_base 后严格复原 T_map_base；输入为非平凡旋转和平移，输出为矩阵逐元素断言结果。
TEST(TfContract, ComposesToMapBase)
{
  const auto map_base = make_transform(Eigen::Vector3d(8.0, -3.0, 1.5), 0.7);
  const auto odom_base = make_transform(Eigen::Vector3d(-1.0, 4.0, 0.2), -0.4);

  const auto map_odom = hdl_localization::calculate_map_to_odom(map_base, odom_base);
  const auto reconstructed_map_base = map_odom * odom_base;

  EXPECT_TRUE(reconstructed_map_base.matrix().isApprox(map_base.matrix(), 1e-12));
}

// 此测试用于验证同一 map 与 odom 基座位姿产生单位 map 到 odom 变换；输入为相同刚体变换，输出为单位矩阵断言结果。
TEST(TfContract, ReturnsIdentityForEqualFrames)
{
  const auto shared_transform = make_transform(Eigen::Vector3d(2.0, 5.0, -0.5), -0.9);

  const auto map_odom = hdl_localization::calculate_map_to_odom(
    shared_transform, shared_transform);

  EXPECT_TRUE(map_odom.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-12));
}

}  // namespace
