// 此文件用于实现 ROS2 双 MID360 点云处理节点的订阅、融合、过滤和兼容话题发布。
#include "livox_cloudpoint_processor/cloud_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

namespace livox_cloudpoint_processor
{
namespace
{

// 此函数用于把单路 Livox CustomMsg 追加到融合点云；输入为消息和目标点云，副作用是写入保持左雷达基准坐标的有效点。
void AppendMessagePoints(const livox_ros_driver2::msg::CustomMsg & message, pcl::PointCloud<pcl::PointXYZI> & merged)
{
  for (const auto & source : message.points) {
    pcl::PointXYZI point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.intensity = static_cast<float>(source.reflectivity);
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) && std::isfinite(point.intensity)) {merged.push_back(point);}
  }
}

}  // namespace

// 此构造函数用于声明参数、创建 ROS2 通信实体并初始化双雷达处理节点；无输入，副作用是建立订阅和发布。
CloudProcessor::CloudProcessor() : Node("threeD_lidar_filter_pointcloud")
{
  DeclareAndReadParameters();
  const auto input_qos = rclcpp::SensorDataQoS().keep_last(1);
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  left_subscription_ = create_subscription<CustomMsg>(left_topic_, input_qos, std::bind(&CloudProcessor::HandleLeftMessage, this, std::placeholders::_1));
  if (enable_dual_lidar_fusion_) {
    right_subscription_ = create_subscription<CustomMsg>(right_topic_, input_qos, std::bind(&CloudProcessor::HandleRightMessage, this, std::placeholders::_1));
  }
  raw_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(raw_topic_, output_qos);
  filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, output_qos);
  grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, output_qos);
  RCLCPP_INFO(get_logger(), "双 MID360 点云处理器已启动：左=%s，右=%s，输出=%s、%s、%s", left_topic_.c_str(), right_topic_.c_str(), raw_topic_.c_str(), filtered_topic_.c_str(), grid_topic_.c_str());
}

// 此函数用于声明并读取节点参数；无输入，副作用是更新话题名、坐标系和处理参数成员。
void CloudProcessor::DeclareAndReadParameters()
{
  left_topic_ = declare_parameter<std::string>("left_topic", "/livox/lidar_192_168_1_3");
  right_topic_ = declare_parameter<std::string>("right_topic", "/livox/lidar_192_168_1_105");
  enable_dual_lidar_fusion_ = declare_parameter<bool>("enable_dual_lidar_fusion", false);
  raw_topic_ = declare_parameter<std::string>("raw_topic", "/lidar_3d");
  filtered_topic_ = declare_parameter<std::string>("filtered_topic", "/filted_topic_3d");
  grid_topic_ = declare_parameter<std::string>("grid_topic", "/grid");
  frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
  parameters_.first_radius = declare_parameter<double>("first_radius", parameters_.first_radius);
  parameters_.second_radius = declare_parameter<double>("second_radius", parameters_.second_radius);
  parameters_.slope_first_radius = declare_parameter<double>("slope_first_radius", parameters_.slope_first_radius);
  parameters_.slope_second_radius = declare_parameter<double>("slope_second_radius", parameters_.slope_second_radius);
  parameters_.slope_third_radius = declare_parameter<double>("slope_third_radius", parameters_.slope_third_radius);
  parameters_.start_height = declare_parameter<double>("start_height", parameters_.start_height);
  parameters_.max_height = declare_parameter<double>("max_height", parameters_.max_height);
  parameters_.slope_1 = declare_parameter<double>("slope_1", parameters_.slope_1);
  parameters_.slope_2 = declare_parameter<double>("slope_2", parameters_.slope_2);
  parameters_.slope_3 = declare_parameter<double>("slope_3", parameters_.slope_3);
  parameters_.dilation_radius = declare_parameter<double>("dilation_radius", parameters_.dilation_radius);
  parameters_.grid_resolution = declare_parameter<double>("grid_resolution", parameters_.grid_resolution);
  parameters_.grid_width = declare_parameter<int>("grid_width", parameters_.grid_width);
  parameters_.grid_height = declare_parameter<int>("grid_height", parameters_.grid_height);
  parameters_.grid_origin_x = declare_parameter<double>("grid_origin_x", parameters_.grid_origin_x);
  parameters_.grid_origin_y = declare_parameter<double>("grid_origin_y", parameters_.grid_origin_y);
}

// 此函数用于接收左侧 MID360 数据；输入为只读 CustomMsg，副作用是更新左侧缓存并尝试触发一次融合。
void CloudProcessor::HandleLeftMessage(const CustomMsg::ConstSharedPtr message)
{
  if (!enable_dual_lidar_fusion_) {
    CustomMsg empty_message;
    ProcessPair(*message, empty_message);
    return;
  }
  {std::lock_guard<std::mutex> lock(cache_mutex_); left_message_ = message; left_ready_ = true;}
  CustomMsg::ConstSharedPtr left, right;
  if (TakeReadyPair(left, right)) {ProcessPair(*left, *right); return;}
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待右侧 MID360 新数据，当前不会发布融合点云。");
}

// 此函数用于接收右侧 MID360 数据；输入为只读 CustomMsg，副作用是更新右侧缓存并尝试触发一次融合。
void CloudProcessor::HandleRightMessage(const CustomMsg::ConstSharedPtr message)
{
  {std::lock_guard<std::mutex> lock(cache_mutex_); right_message_ = message; right_ready_ = true;}
  CustomMsg::ConstSharedPtr left, right;
  if (TakeReadyPair(left, right)) {ProcessPair(*left, *right); return;}
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待左侧 MID360 新数据，当前不会发布融合点云。");
}

// 此函数用于在两路都有新帧时提取缓存；输出为是否获得可处理数据，副作用是消费两路新帧标记。
bool CloudProcessor::TakeReadyPair(CustomMsg::ConstSharedPtr & left, CustomMsg::ConstSharedPtr & right)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!left_ready_ || !right_ready_) {return false;}
  left = left_message_; right = right_message_; left_ready_ = false; right_ready_ = false;
  return true;
}

// 此函数用于融合一对 Livox CustomMsg；输入为左右消息，输出为完成历史固定偏移后的 PointXYZI 点云。
pcl::PointCloud<pcl::PointXYZI> CloudProcessor::MergeMessages(const CustomMsg & left, const CustomMsg & right) const
{
  pcl::PointCloud<pcl::PointXYZI> merged;
  merged.reserve(left.points.size() + right.points.size());
  AppendMessagePoints(left, merged); AppendMessagePoints(right, merged);
  merged.width = static_cast<std::uint32_t>(merged.size()); merged.height = 1; merged.is_dense = false;
  return merged;
}

// 此函数用于执行径向筛选、PCL 下采样、离群去除和法向量筛选；输入为融合点云，输出为过滤后的 PointXYZI 点云。
pcl::PointCloud<pcl::PointXYZI> CloudProcessor::FilterCloud(const pcl::PointCloud<pcl::PointXYZI> & raw_cloud) const
{
  pcl::PointCloud<pcl::PointXYZI> radial;
  for (const auto & point : raw_cloud.points) {if (ShouldKeepPoint(point, parameters_)) {radial.push_back(point);}}
  if (radial.empty()) {return radial;}
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(radial, *xyz);
  pcl::VoxelGrid<pcl::PointXYZ> voxel; voxel.setInputCloud(xyz); voxel.setLeafSize(0.05F, 0.05F, 0.05F);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>()); voxel.filter(*downsampled);
  if (downsampled->empty()) {return {};}
  pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZ>());
  if (downsampled->size() >= 50U) {pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter; filter.setInputCloud(downsampled); filter.setMeanK(50); filter.setStddevMulThresh(1.0); filter.filter(*denoised);} else {*denoised = *downsampled;}
  if (denoised->empty()) {return {};}
  pcl::PointCloud<pcl::PointXYZ> vertical;
  if (denoised->size() >= 20U) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normals_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    normals_estimator.setInputCloud(denoised); normals_estimator.setSearchMethod(tree); normals_estimator.setKSearch(20); normals_estimator.setNumberOfThreads(8); normals_estimator.compute(*normals);
    constexpr float kHalfPi = 1.57079632679F; constexpr float kTolerance = 0.78539816339F;
    for (std::size_t i = 0; i < denoised->size(); ++i) {const auto & n = normals->points[i]; const float norm = std::sqrt(n.normal_x*n.normal_x+n.normal_y*n.normal_y+n.normal_z*n.normal_z); if (std::isfinite(norm) && norm > 1.0e-6F && std::abs(std::acos(std::clamp(n.normal_z/norm, -1.0F, 1.0F)) - kHalfPi) < kTolerance) {vertical.push_back(denoised->points[i]);}}
  } else {vertical = *denoised;}
  pcl::PointCloud<pcl::PointXYZI> result; pcl::copyPointCloud(vertical, result); result.width = static_cast<std::uint32_t>(result.size()); result.height = 1; result.is_dense = false;
  return result;
}

// 此函数用于处理一对输入消息并发布三类兼容输出；副作用是发布 /lidar_3d、/filted_topic_3d 和 /grid。
void CloudProcessor::ProcessPair(const CustomMsg & left, const CustomMsg & right)
{
  const builtin_interfaces::msg::Time stamp = now();
  const auto raw = MergeMessages(left, right); const auto filtered = FilterCloud(raw);
  sensor_msgs::msg::PointCloud2 raw_message; pcl::toROSMsg(raw, raw_message); raw_message.header.frame_id = frame_id_; raw_message.header.stamp = stamp; raw_publisher_->publish(raw_message);
  sensor_msgs::msg::PointCloud2 filtered_message; pcl::toROSMsg(filtered, filtered_message); filtered_message.header.frame_id = frame_id_; filtered_message.header.stamp = stamp; filtered_publisher_->publish(filtered_message);
  auto grid = BuildOccupancyGrid(filtered, parameters_, frame_id_, stamp); InflateOccupancyGrid(grid, parameters_.dilation_radius); ProcessVisibility(grid); grid_publisher_->publish(grid);
}

}  // namespace livox_cloudpoint_processor