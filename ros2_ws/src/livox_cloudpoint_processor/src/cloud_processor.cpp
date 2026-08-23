// 此文件用于实现 ROS2 双 MID360 点云处理节点的订阅、融合、过滤和兼容话题发布。
#include "livox_cloudpoint_processor/cloud_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <limits>
#include <vector>
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

// 此函数用于将 ROS Header 时间转换为纳秒；输入为 Header，输出为无符号纳秒时间，无 ROS 通信副作用。
std::uint64_t HeaderStampNs(const std_msgs::msg::Header & header)
{
  return static_cast<std::uint64_t>(header.stamp.sec) * 1000000000ULL +
    static_cast<std::uint64_t>(header.stamp.nanosec);
}

// 此函数用于取得 Livox 消息的逐点时间基准；输入为 CustomMsg，输出为 timebase 或 Header 对应的纳秒时间，无 ROS 通信副作用。
std::uint64_t ResolveBaseTimeNs(const livox_ros_driver2::msg::CustomMsg & message)
{
  return message.timebase == 0U ? HeaderStampNs(message.header) : message.timebase;
}

// 此函数用于按给定旋转和平移变换一点；输入为源点和右雷达外参，输出为左雷达基准坐标的点，无 ROS 通信副作用。
pcl::PointXYZI TransformPoint(
  const livox_ros_driver2::msg::CustomPoint & source,
  const std::array<double, 9> & rotation,
  const std::array<double, 3> & translation)
{
  pcl::PointXYZI point;
  point.x = static_cast<float>(rotation[0] * source.x + rotation[1] * source.y + rotation[2] * source.z + translation[0]);
  point.y = static_cast<float>(rotation[3] * source.x + rotation[4] * source.y + rotation[5] * source.z + translation[1]);
  point.z = static_cast<float>(rotation[6] * source.x + rotation[7] * source.y + rotation[8] * source.z + translation[2]);
  point.intensity = static_cast<float>(source.reflectivity);
  return point;
}

// 此函数用于把一路 Livox CustomMsg 追加到 PCL 点云；输入为消息和对应外参，副作用是写入左雷达基准下的有效点。
void AppendMessagePoints(
  const livox_ros_driver2::msg::CustomMsg & message,
  const std::array<double, 9> & rotation,
  const std::array<double, 3> & translation,
  pcl::PointCloud<pcl::PointXYZI> & merged)
{
  for (const auto & source : message.points) {
    const auto point = TransformPoint(source, rotation, translation);
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) && std::isfinite(point.intensity)) {
      merged.push_back(point);
    }
  }
}

// 此函数用于把一路点按统一时间基准追加到融合 CustomMsg；输入为消息、统一基准和外参，副作用是写入重建 offset_time 的有效点。
void AppendRebasedPoints(
  const livox_ros_driver2::msg::CustomMsg & message,
  const std::uint64_t unified_base_ns,
  const std::array<double, 9> & rotation,
  const std::array<double, 3> & translation,
  livox_ros_driver2::msg::CustomMsg & fused)
{
  const auto source_base_ns = ResolveBaseTimeNs(message);
  for (const auto & source : message.points) {
    const auto absolute_ns = source_base_ns + static_cast<std::uint64_t>(source.offset_time);
    if (absolute_ns < unified_base_ns || absolute_ns - unified_base_ns > std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }
    const auto transformed = TransformPoint(source, rotation, translation);
    if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y) || !std::isfinite(transformed.z)) {
      continue;
    }
    livox_ros_driver2::msg::CustomPoint point;
    point.offset_time = static_cast<std::uint32_t>(absolute_ns - unified_base_ns);
    point.x = transformed.x;
    point.y = transformed.y;
    point.z = transformed.z;
    point.reflectivity = source.reflectivity;
    point.tag = source.tag;
    point.line = source.line;
    fused.points.push_back(point);
  }
}

}  // namespace

// 此构造函数用于声明参数、创建 ROS2 通信实体并初始化双雷达处理节点；无输入，副作用是建立订阅和发布。
CloudProcessor::CloudProcessor() : Node("threeD_lidar_filter_pointcloud")
{
  DeclareAndReadParameters();
  const auto input_qos = rclcpp::SensorDataQoS().keep_last(std::max(1, input_qos_depth_));
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  left_subscription_ = create_subscription<CustomMsg>(left_topic_, input_qos, std::bind(&CloudProcessor::HandleLeftMessage, this, std::placeholders::_1));
  if (enable_dual_lidar_fusion_) {
    right_subscription_ = create_subscription<CustomMsg>(right_topic_, input_qos, std::bind(&CloudProcessor::HandleRightMessage, this, std::placeholders::_1));
    fused_custom_publisher_ = create_publisher<CustomMsg>(fused_custom_topic_, input_qos);
  }
  raw_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(raw_topic_, output_qos);
  filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, output_qos);
  grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, output_qos);
  RCLCPP_INFO(get_logger(), "双 MID360 点云处理器已启动：左=%s，右=%s，双融合=%s，输出=%s、%s、%s", left_topic_.c_str(), right_topic_.c_str(), enable_dual_lidar_fusion_ ? "开启" : "关闭", raw_topic_.c_str(), filtered_topic_.c_str(), grid_topic_.c_str());
}

// 此函数用于声明并读取节点参数；无输入，副作用是更新话题名、坐标系和处理参数成员。
void CloudProcessor::DeclareAndReadParameters()
{
  left_topic_ = declare_parameter<std::string>("left_topic", "/livox/lidar_192_168_1_3");
  right_topic_ = declare_parameter<std::string>("right_topic", "/livox/lidar_192_168_1_105");
  enable_dual_lidar_fusion_ = declare_parameter<bool>("enable_dual_lidar_fusion", false);
  fused_custom_topic_ = declare_parameter<std::string>("fused_custom_topic", "/livox/fused_custom");
  dual_sync_tolerance_ms_ = declare_parameter<double>("dual_sync_tolerance_ms", 10.0);
  dual_cache_seconds_ = declare_parameter<double>("dual_cache_seconds", 0.2);
  dual_drop_policy_ = declare_parameter<std::string>("dual_drop_policy", "drop_older");
  input_qos_depth_ = declare_parameter<int>("input_qos_depth", 1);
  const auto translation = declare_parameter<std::vector<double>>("right_extrinsic_translation", {0.0, 0.0, 0.0});
  const auto rotation = declare_parameter<std::vector<double>>("right_extrinsic_rotation", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  if (translation.size() == right_extrinsic_translation_.size()) {
    std::copy(translation.begin(), translation.end(), right_extrinsic_translation_.begin());
  } else {
    RCLCPP_WARN(get_logger(), "右雷达平移外参长度不是 3，使用单位外参");
  }
  if (rotation.size() == right_extrinsic_rotation_.size()) {
    std::copy(rotation.begin(), rotation.end(), right_extrinsic_rotation_.begin());
  } else {
    RCLCPP_WARN(get_logger(), "右雷达旋转外参长度不是 9，使用单位外参");
  }
  if (dual_drop_policy_ != "drop_older" && dual_drop_policy_ != "drop_pair") {
    RCLCPP_WARN(get_logger(), "未知双雷达丢帧策略 %s，改用 drop_older", dual_drop_policy_.c_str());
    dual_drop_policy_ = "drop_older";
  }
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
  {std::lock_guard<std::mutex> lock(cache_mutex_); left_message_ = message; left_received_ = now(); left_ready_ = true;}
  CustomMsg::ConstSharedPtr left, right;
  if (TakeReadyPair(left, right)) {ProcessPair(*left, *right); return;}
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待右侧 MID360 新数据，当前不会发布融合点云。");
}

// 此函数用于接收右侧 MID360 数据；输入为只读 CustomMsg，副作用是更新右侧缓存并尝试触发一次融合。
void CloudProcessor::HandleRightMessage(const CustomMsg::ConstSharedPtr message)
{
  {std::lock_guard<std::mutex> lock(cache_mutex_); right_message_ = message; right_received_ = now(); right_ready_ = true;}
  CustomMsg::ConstSharedPtr left, right;
  if (TakeReadyPair(left, right)) {ProcessPair(*left, *right); return;}
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待左侧 MID360 新数据，当前不会发布融合点云。");
}

// 此函数用于按缓存时长、时间差和丢帧策略选取同步帧；输出为是否获得可处理数据，副作用是消费或丢弃缓存帧。
bool CloudProcessor::TakeReadyPair(CustomMsg::ConstSharedPtr & left, CustomMsg::ConstSharedPtr & right)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  const auto receipt_now = now();
  if (left_ready_ && (receipt_now - left_received_).seconds() > dual_cache_seconds_) {
    left_ready_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "左 MID360 缓存超时，按参数丢弃旧帧");
  }
  if (right_ready_ && (receipt_now - right_received_).seconds() > dual_cache_seconds_) {
    right_ready_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "右 MID360 缓存超时，按参数丢弃旧帧");
  }
  if (!left_ready_ || !right_ready_) {
    return false;
  }
  if (!IsPairSynchronized(*left_message_, *right_message_)) {
    if (dual_drop_policy_ == "drop_pair") {
      left_ready_ = false;
      right_ready_ = false;
    } else if (MessageBaseTimeNs(*left_message_) <= MessageBaseTimeNs(*right_message_)) {
      left_ready_ = false;
    } else {
      right_ready_ = false;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "双 MID360 时间差超过 %.3f ms，按 %s 策略丢帧", dual_sync_tolerance_ms_, dual_drop_policy_.c_str());
    return false;
  }
  left = left_message_;
  right = right_message_;
  left_ready_ = false;
  right_ready_ = false;
  return true;
}

// 此函数用于取得 CustomMsg 的统一纳秒基准时间；输入为消息，输出为 timebase 或 header 时间对应的纳秒值，无 ROS 通信副作用。
std::uint64_t CloudProcessor::MessageBaseTimeNs(const CustomMsg & message) const
{
  return ResolveBaseTimeNs(message);
}

// 此函数用于判断左右帧是否在参数化同步阈值内；输入为两路消息，输出为同步判断，无 ROS 通信副作用。
bool CloudProcessor::IsPairSynchronized(const CustomMsg & left, const CustomMsg & right) const
{
  const auto left_ns = MessageBaseTimeNs(left);
  const auto right_ns = MessageBaseTimeNs(right);
  const auto delta_ns = left_ns >= right_ns ? left_ns - right_ns : right_ns - left_ns;
  return delta_ns <= static_cast<std::uint64_t>(std::max(0.0, dual_sync_tolerance_ms_) * 1000000.0);
}

// 此函数用于按右雷达外参将两路点重建为一个统一 timebase 的 CustomMsg；输入为同步左右帧，输出为可供未来 Point-LIO 使用的融合消息。
CloudProcessor::CustomMsg CloudProcessor::BuildFusedCustomMessage(const CustomMsg & left, const CustomMsg & right) const
{
  CustomMsg fused;
  const auto unified_base_ns = std::min(MessageBaseTimeNs(left), MessageBaseTimeNs(right));
  fused.header = left.header;
  fused.header.frame_id = frame_id_;
  fused.header.stamp.sec = static_cast<std::int32_t>(unified_base_ns / 1000000000ULL);
  fused.header.stamp.nanosec = static_cast<std::uint32_t>(unified_base_ns % 1000000000ULL);
  fused.timebase = unified_base_ns;
  fused.lidar_id = left.lidar_id;
  fused.rsvd = left.rsvd;
  fused.points.reserve(left.points.size() + right.points.size());
  constexpr std::array<double, 9> identity_rotation{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  constexpr std::array<double, 3> zero_translation{{0.0, 0.0, 0.0}};
  AppendRebasedPoints(left, unified_base_ns, identity_rotation, zero_translation, fused);
  AppendRebasedPoints(right, unified_base_ns, right_extrinsic_rotation_, right_extrinsic_translation_, fused);
  std::sort(fused.points.begin(), fused.points.end(), [](const auto & lhs, const auto & rhs) {return lhs.offset_time < rhs.offset_time;});
  fused.point_num = static_cast<std::uint32_t>(fused.points.size());
  return fused;
}

// 此函数用于把单路或已重建的 CustomMsg 转换为 PointXYZI；输入为消息，输出为点云，无 ROS 通信副作用。
pcl::PointCloud<pcl::PointXYZI> CloudProcessor::MergeMessages(const CustomMsg & left, const CustomMsg & right) const
{
  pcl::PointCloud<pcl::PointXYZI> merged;
  merged.reserve(left.points.size() + right.points.size());
  constexpr std::array<double, 9> identity_rotation{{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  constexpr std::array<double, 3> zero_translation{{0.0, 0.0, 0.0}};
  AppendMessagePoints(left, identity_rotation, zero_translation, merged);
  if (!right.points.empty()) {
    AppendMessagePoints(right, right_extrinsic_rotation_, right_extrinsic_translation_, merged);
  }
  merged.width = static_cast<std::uint32_t>(merged.size());
  merged.height = 1;
  merged.is_dense = false;
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

// 此函数用于处理一对输入消息并发布三类兼容输出；启用双融合时还发布重建时间基准后的 CustomMsg，副作用是 ROS2 发布。
void CloudProcessor::ProcessPair(const CustomMsg & left, const CustomMsg & right)
{
  CustomMsg empty_message;
  CustomMsg fused_message;
  const CustomMsg * cloud_message = &left;
  if (enable_dual_lidar_fusion_) {
    fused_message = BuildFusedCustomMessage(left, right);
    if (fused_message.points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "融合 CustomMsg 没有有效点，本帧不发布");
      return;
    }
    fused_custom_publisher_->publish(fused_message);
    cloud_message = &fused_message;
  }
  const auto raw = MergeMessages(*cloud_message, empty_message);
  const auto filtered = FilterCloud(raw);
  const auto stamp = cloud_message->header.stamp;
  sensor_msgs::msg::PointCloud2 raw_message;
  pcl::toROSMsg(raw, raw_message);
  raw_message.header.frame_id = frame_id_;
  raw_message.header.stamp = stamp;
  raw_publisher_->publish(raw_message);
  sensor_msgs::msg::PointCloud2 filtered_message;
  pcl::toROSMsg(filtered, filtered_message);
  filtered_message.header.frame_id = frame_id_;
  filtered_message.header.stamp = stamp;
  filtered_publisher_->publish(filtered_message);
  auto grid = BuildOccupancyGrid(filtered, parameters_, frame_id_, stamp);
  InflateOccupancyGrid(grid, parameters_.dilation_radius);
  ProcessVisibility(grid);
  grid_publisher_->publish(grid);
}

}  // namespace livox_cloudpoint_processor