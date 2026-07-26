// 此文件用于声明 ROS2 双 MID360 点云处理节点及其消息缓存、融合和发布职责。
#ifndef LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_PROCESSOR_HPP_
#define LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_PROCESSOR_HPP_

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
  // 此函数用于在两路都有新帧时提取缓存；输出为是否获得可处理数据，副作用是消费两路新帧标记。
  bool TakeReadyPair(CustomMsg::ConstSharedPtr & left, CustomMsg::ConstSharedPtr & right);
  // 此函数用于融合一对 Livox CustomMsg；输入为左右消息，输出为完成历史固定偏移后的 PointXYZI 点云。
  pcl::PointCloud<pcl::PointXYZI> MergeMessages(const CustomMsg & left, const CustomMsg & right) const;
  // 此函数用于执行径向筛选、PCL 下采样、离群去除和法向量筛选；输入为融合点云，输出为过滤后的 PointXYZI 点云。
  pcl::PointCloud<pcl::PointXYZI> FilterCloud(const pcl::PointCloud<pcl::PointXYZI> & raw_cloud) const;
  // 此函数用于处理一对输入消息并发布三类兼容输出；副作用是发布 /3Dlidar、/filted_topic_3d 和 /grid。
  void ProcessPair(const CustomMsg & left, const CustomMsg & right);

  CloudProcessingParameters parameters_;
  std::string left_topic_, right_topic_, raw_topic_, filtered_topic_, grid_topic_, frame_id_;
  std::mutex cache_mutex_;
  CustomMsg::ConstSharedPtr left_message_, right_message_;
  bool left_ready_{false}, right_ready_{false};
  rclcpp::Subscription<CustomMsg>::SharedPtr left_subscription_, right_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_publisher_, filtered_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
};

}  // namespace livox_cloudpoint_processor
#endif  // LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_PROCESSOR_HPP_