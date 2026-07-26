// 此文件用于实现双 MID360 点云的连续分段过滤、占据栅格膨胀和安全可见性处理。
#include "livox_cloudpoint_processor/cloud_algorithms.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace livox_cloudpoint_processor
{
namespace
{

// 此函数用于从栅格中心沿 Bresenham 射线标记障碍物后的不可见区域；输入为栅格和目标格坐标，副作用是原地写入未知值。
void MarkOccludedCellsOnRay(nav_msgs::msg::OccupancyGrid & grid, int target_x, int target_y)
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  int x = width / 2;
  int y = height / 2;
  const int dx = std::abs(target_x - x);
  const int dy = -std::abs(target_y - y);
  const int step_x = x < target_x ? 1 : -1;
  const int step_y = y < target_y ? 1 : -1;
  int error = dx + dy;
  bool blocked = false;
  while (true) {
    const int index = y * width + x;
    if (blocked && grid.data[index] != 100) {grid.data[index] = -1;}
    if (grid.data[index] == 100) {blocked = true;}
    if (x == target_x && y == target_y) {break;}
    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {error += dy; x += step_x;}
    if (doubled_error <= dx) {error += dx; y += step_y;}
  }
}

}  // namespace

// 此函数用于计算给定平面半径处的连续分段高度阈值；输入为半径和参数，输出为确定的高度上限。
double CalculateGroundHeight(double radius, const CloudProcessingParameters & p)
{
  if (radius <= p.second_radius) {return p.start_height;}
  const double h1 = std::min(p.max_height, p.start_height + (p.slope_first_radius - p.second_radius) * p.slope_1);
  const double h2 = std::min(p.max_height, h1 + (p.slope_second_radius - p.slope_first_radius) * p.slope_2);
  const double h3 = std::min(p.max_height, h2 + (p.slope_third_radius - p.slope_second_radius) * p.slope_3);
  if (radius <= p.slope_first_radius) {return std::min(p.max_height, p.start_height + (radius - p.second_radius) * p.slope_1);}
  if (radius <= p.slope_second_radius) {return std::min(p.max_height, h1 + (radius - p.slope_first_radius) * p.slope_2);}
  if (radius <= p.slope_third_radius) {return std::min(p.max_height, h2 + (radius - p.slope_second_radius) * p.slope_3);}
  return h3;
}

// 此函数用于判断一个点是否保留到后续过滤流程；输入为点和参数，输出为是否保留，并保留历史固定坐标偏移语义。
bool ShouldKeepPoint(const pcl::PointXYZI & point, const CloudProcessingParameters & p)
{
  const double x = static_cast<double>(point.x) + 0.011;
  const double y = static_cast<double>(point.y) + 0.17166;
  const double radius = std::hypot(x, y);
  return radius > p.first_radius && static_cast<double>(point.z) <= CalculateGroundHeight(radius, p);
}

// 此函数用于根据过滤点云构造二维占据栅格；输入为点云、参数、坐标系和时间戳，输出为未膨胀的 OccupancyGrid。
nav_msgs::msg::OccupancyGrid BuildOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI> & cloud, const CloudProcessingParameters & p, const std::string & frame_id, const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.header.stamp = stamp;
  grid.info.resolution = p.grid_resolution;
  grid.info.width = static_cast<std::uint32_t>(p.grid_width);
  grid.info.height = static_cast<std::uint32_t>(p.grid_height);
  grid.info.origin.position.x = p.grid_origin_x;
  grid.info.origin.position.y = p.grid_origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(p.grid_width * p.grid_height), 0);
  for (const auto & point : cloud.points) {
    const int x = static_cast<int>((point.x - p.grid_origin_x) / p.grid_resolution);
    const int y = static_cast<int>((point.y - p.grid_origin_y) / p.grid_resolution);
    if (x >= 0 && x < p.grid_width && y >= 0 && y < p.grid_height) {grid.data[static_cast<std::size_t>(y * p.grid_width + x)] = 100;}
  }
  return grid;
}

// 此函数用于按照机器人半径膨胀栅格障碍物；输入输出均为同一栅格对象，副作用是更新占据代价。
void InflateOccupancyGrid(nav_msgs::msg::OccupancyGrid & grid, double robot_radius)
{
  if (grid.info.resolution <= 0.0 || robot_radius <= 0.0) {return;}
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const int radius = static_cast<int>(std::ceil(robot_radius / grid.info.resolution));
  const std::vector<int8_t> original = grid.data;
  std::vector<int8_t> inflated = original;
  for (int oy = 0; oy < height; ++oy) {
    for (int ox = 0; ox < width; ++ox) {
      if (original[oy * width + ox] <= 0) {continue;}
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int x = ox + dx; const int y = oy + dy;
          if (x < 0 || x >= width || y < 0 || y >= height) {continue;}
          const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (distance > radius) {continue;}
          const int cost = std::max(1, static_cast<int>(100.0 * (1.0 - distance / radius)));
          inflated[y * width + x] = std::max(inflated[y * width + x], static_cast<int8_t>(cost));
        }
      }
    }
  }
  grid.data = std::move(inflated);
}

// 此函数用于从栅格中心向边界做可见性射线处理；输入输出均为同一栅格对象，副作用是将障碍物后的区域标为未知。
void ProcessVisibility(nav_msgs::msg::OccupancyGrid & grid)
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  if (width <= 0 || height <= 0 || grid.data.empty()) {return;}
  for (int x = 0; x < width; ++x) {MarkOccludedCellsOnRay(grid, x, 0); MarkOccludedCellsOnRay(grid, x, height - 1);}
  for (int y = 0; y < height; ++y) {MarkOccludedCellsOnRay(grid, 0, y); MarkOccludedCellsOnRay(grid, width - 1, y);}
}

}  // namespace livox_cloudpoint_processor