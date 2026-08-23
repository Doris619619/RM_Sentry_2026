// 此文件用于定义 HDL 将 T_map_base 与 Point-LIO 的 T_odom_base 转换为唯一 T_map_odom 的纯计算契约。
#pragma once

#include <Eigen/Geometry>

namespace hdl_localization {

// 此函数用于计算唯一的 map 到 odom 变换；输入为同一时刻的 T_map_base 和 T_odom_base，输出为 T_map_odom，不产生 ROS 通信副作用。
inline Eigen::Isometry3d calculate_map_to_odom(
  const Eigen::Isometry3d& map_base,
  const Eigen::Isometry3d& odom_base)
{
  return map_base * odom_base.inverse();
}

}  // namespace hdl_localization
