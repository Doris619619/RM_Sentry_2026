// 此文件用于声明 ROS2 双 MID360 点云处理节点及其消息缓存、融合和发布职责。
#ifndef LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_PROCESSOR_HPP_
#define LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_PROCESSOR_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "livox_cloudpoint_processor/cloud_algorithms.hpp"

namespace livox_cloudpoint_processor
{

// 此类用于订阅两台 MID360 的 CustomMsg、融合并发布兼容旧系统的点云和栅格话题。
class CloudProcessor final : public rclcpp::Node
{
public:
  // 此构造函数用于声明参数、创建 ROS2 通信实体并初始化双雷达处理节点；无输入，副作用是建立订阅和发布。
  CloudProcessor();

private:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;

  // 此函数用于声明并读取节点参数；无输入，副作用是更新话题名、坐标系和处理参数成员。
  void DeclareAndReadParameters();
  // 此函数用于接收左侧 MID360 数据；输入为只读 CustomMsg，副作用是更新左侧缓存并尝试触发一次融合。
  void HandleLeftMessage(const CustomMsg::ConstSharedPtr message);
  // 此函数用于接收右侧 MID360 数据；输入为只读 CustomMsg，副作用是更新右侧缓存并尝试触发一次融合。
  void HandleRightMessage(const CustomMsg::ConstSharedPtr message);
  // 此函数用于按缓存时长、时间差和丢帧策略选取同步帧；输出为是否获得可处理数据，副作用是消费或丢弃缓存帧。
  bool TakeReadyPair(CustomMsg::ConstSharedPtr & left, CustomMsg::ConstSharedPtr & right);
  // 此函数用于取得 CustomMsg 的统一纳秒基准时间；输入为消息，输出为 timebase 或 header 时间对应的纳秒值，无 ROS 通信副作用。
  std::uint64_t MessageBaseTimeNs(const CustomMsg & message) const;
  // 此函数用于判断左右帧是否在参数化同步阈值内；输入为两路消息，输出为同步判断，无 ROS 通信副作用。
  bool IsPairSynchronized(const CustomMsg & left, const CustomMsg & right) const;
  // 此函数用于按右雷达外参将两路点重建为一个统一 timebase 的 CustomMsg；输入为同步左右帧，输出为可供未来 Point-LIO 使用的融合消息。
  CustomMsg BuildFusedCustomMessage(const CustomMsg & left, const CustomMsg & right) const;
  // 此函数用于把单路或已重建的 CustomMsg 转换为 PointXYZI；输入为消息，输出为点云，无 ROS 通信副作用。
  pcl::PointCloud<pcl::PointXYZI> MergeMessages(const CustomMsg & left, const CustomMsg & right) const;
  // 此函数用于执行径向筛选、PCL 下采样、离群去除和法向量筛选；输入为融合点云，输出为过滤后的 PointXYZI 点云。
  pcl::PointCloud<pcl::PointXYZI> FilterCloud(const pcl::PointCloud<pcl::PointXYZI> & raw_cloud) const;
  // 此函数用于处理一对输入消息并发布三类兼容输出；副作用是发布 /lidar_3d、/filted_topic_3d 和 /grid。
  void ProcessPair(const CustomMsg & left, const CustomMsg & right);

  CloudProcessingParameters parameters_;
  std::string left_topic_, right_topic_, raw_topic_, filtered_topic_, grid_topic_, fused_custom_topic_, frame_id_;
  std::string dual_drop_policy_{"drop_older"};
  std::array<double, 3> right_extrinsic_translation_{{0.0, 0.0, 0.0}};
  std::array<double, 9> right_extrinsic_rotation_{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  double dual_sync_tolerance_ms_{10.0};
  double dual_cache_seconds_{0.2};
  int input_qos_depth_{1};
  std::mutex cache_mutex_;
  CustomMsg::ConstSharedPtr left_message_, right_message_;
  rclcpp::Time left_received_, right_received_;
  bool enable_dual_lidar_fusion_{false};
  bool left_ready_{false}, right_ready_{false};
  rclcpp::Subscription<CustomMsg>::SharedPtr left_subscription_, right_subscription_;
  rclcpp::Publisher<CustomMsg>::SharedPtr fused_custom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_publisher_, filtered_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
};

}  // namespace livox_cloudpoint_processor
#endif  // LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_PROCESSOR_HPP_