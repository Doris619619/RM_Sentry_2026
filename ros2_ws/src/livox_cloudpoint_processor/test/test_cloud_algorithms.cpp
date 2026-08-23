// 此文件用于验证连续分段过滤与占据栅格算法在 ROS2 下具有确定性输出。
#include <gtest/gtest.h>
#include "livox_cloudpoint_processor/cloud_algorithms.hpp"

namespace livox_cloudpoint_processor
{

// 此测试用于验证连续分段高度在每个半径边界处单调且第三半径参与计算。
TEST(CloudAlgorithms, GroundHeightUsesAllConfiguredSegments)
{
  CloudProcessingParameters parameters;
  const double first = CalculateGroundHeight(parameters.slope_first_radius, parameters);
  const double second = CalculateGroundHeight(parameters.slope_second_radius, parameters);
  const double third = CalculateGroundHeight(parameters.slope_third_radius, parameters);
  EXPECT_GE(first, parameters.start_height);
  EXPECT_GE(second, first);
  EXPECT_GE(third, second);
  EXPECT_DOUBLE_EQ(CalculateGroundHeight(parameters.slope_third_radius + 1.0, parameters), third);
}

// 此测试用于验证车体半径内点被排除且有效外部低点可进入后续处理。
TEST(CloudAlgorithms, RadialFilterRejectsVehicleBodyAndKeepsExternalLowPoint)
{
  CloudProcessingParameters parameters;
  pcl::PointXYZI vehicle; vehicle.x = 0.0F; vehicle.y = 0.0F; vehicle.z = 0.0F;
  EXPECT_FALSE(ShouldKeepPoint(vehicle, parameters));
  pcl::PointXYZI external; external.x = 1.0F; external.y = 0.0F; external.z = 0.05F;
  EXPECT_TRUE(ShouldKeepPoint(external, parameters));
}

// 此测试用于验证栅格尺寸、原点和中心点占据状态符合旧接口约定。
TEST(CloudAlgorithms, OccupancyGridKeepsLegacyGeometry)
{
  CloudProcessingParameters parameters;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI point; point.x = 0.0F; point.y = 0.0F; cloud.push_back(point);
  builtin_interfaces::msg::Time stamp;
  const auto grid = BuildOccupancyGrid(cloud, parameters, "aft_mapped", stamp);
  EXPECT_EQ(grid.info.width, 200U);
  EXPECT_EQ(grid.info.height, 200U);
  EXPECT_NEAR(grid.info.resolution, 0.05F, 1.0e-6F);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -5.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, -5.0);
  EXPECT_EQ(grid.data[100 * 200 + 100], 100);
}

// 此测试用于验证障碍物膨胀会提高邻近栅格代价。
TEST(CloudAlgorithms, InflationCreatesNeighbourCost)
{
  CloudProcessingParameters parameters;
  parameters.grid_width = 5; parameters.grid_height = 5; parameters.grid_resolution = 1.0;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI point; point.x = -3.0F; point.y = -3.0F; cloud.push_back(point);
  builtin_interfaces::msg::Time stamp;
  auto grid = BuildOccupancyGrid(cloud, parameters, "aft_mapped", stamp);
  InflateOccupancyGrid(grid, 1.0);
  EXPECT_EQ(grid.data[2 * 5 + 2], 100);
  EXPECT_GT(grid.data[2 * 5 + 3], 0);
}

}  // namespace livox_cloudpoint_processor