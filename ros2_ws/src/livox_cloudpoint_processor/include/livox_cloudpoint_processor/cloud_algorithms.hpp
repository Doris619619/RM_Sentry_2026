// 此文件用于声明双 MID360 点云过滤、栅格生成、膨胀和可见性处理的无状态算法。
#ifndef LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_ALGORITHMS_HPP_
#define LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_ALGORITHMS_HPP_

#include <string>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace livox_cloudpoint_processor
{

// 此结构用于集中保存连续分段地面过滤和栅格输出的运行参数。
struct CloudProcessingParameters
{
  double first_radius{0.35};
  double second_radius{0.35};
  double slope_first_radius{0.55};
  double slope_second_radius{0.80};
  double slope_third_radius{0.90};
  double start_height{0.10};
  double max_height{0.30};
  double slope_1{0.50};
  double slope_2{0.10};
  double slope_3{0.10};
  double dilation_radius{1.20};
  double grid_resolution{0.05};
  int grid_width{200};
  int grid_height{200};
  double grid_origin_x{-5.0};
  double grid_origin_y{-5.0};
};

// 此函数用于计算给定平面半径处的连续分段高度阈值；输入为半径和参数，输出为确定的高度上限。
double CalculateGroundHeight(double radius, const CloudProcessingParameters & parameters);

// 此函数用于判断一个点是否保留到后续过滤流程；输入为点和参数，输出为是否保留，并保留历史固定坐标偏移语义。
bool ShouldKeepPoint(const pcl::PointXYZI & point, const CloudProcessingParameters & parameters);

// 此函数用于根据过滤点云构造二维占据栅格；输入为点云、参数、坐标系和时间戳，输出为未膨胀的 OccupancyGrid。
nav_msgs::msg::OccupancyGrid BuildOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI> & cloud, const CloudProcessingParameters & parameters, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp);

// 此函数用于按照机器人半径膨胀栅格障碍物；输入输出均为同一栅格对象，副作用是更新占据代价。
void InflateOccupancyGrid(nav_msgs::msg::OccupancyGrid & grid, double robot_radius);

// 此函数用于从栅格中心向边界做可见性射线处理；输入输出均为同一栅格对象，副作用是将障碍物后的区域标为未知。
void ProcessVisibility(nav_msgs::msg::OccupancyGrid & grid);

}  // namespace livox_cloudpoint_processor
#endif  // LIVOX_CLOUDPOINT_PROCESSOR__CLOUD_ALGORITHMS_HPP_