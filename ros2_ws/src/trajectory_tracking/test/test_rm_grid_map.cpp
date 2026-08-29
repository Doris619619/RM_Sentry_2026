#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <trajectory_tracking/RM_GridMap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

TEST(TrackingGridMap, imports_map_frame_cloud_as_dynamic_obstacle) {
  const auto maps = ament_index_cpp::get_package_share_directory("trajectory_generation") + "/map";
  ros::NodeHandle parameters;
  parameters.setParam("trajectory_generator/height_bias", 0.015294117853045464);
  parameters.setParam("trajectory_generator/height_interval", 1.5);
  parameters.setParam("trajectory_generator/height_threshold", 0.08);
  parameters.setParam("trajectory_generator/height_sencond_high_threshold", 0.2);

  TrackingGridMap map;
  map.initGridMap(parameters, maps + "/occfinal.png", maps + "/bevfinal.png", maps + "/occtopo.png",
                  0.05, Eigen::Vector3d(-13.394, -12.079, 0.0), Eigen::Vector3d(6.606, 7.921, 2.0),
                  400, 400, 40, 0.35, -0.05, 1.2, 6.0);
  ASSERT_NE(map.data, nullptr);

  const Eigen::Vector3d obstacle(-3.50, 1.45, 0.0);
  const auto index = map.coord2gridIndex(obstacle);
  ASSERT_FALSE(map.isStaticOccupied(index, false));
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(static_cast<float>(obstacle.x()), static_cast<float>(obstacle.y()),
                                static_cast<float>(map.getHeight(index.x(), index.y()) + 0.10)));
  map.localPointCloudToObstacle(cloud, true, Eigen::Vector3d(-3.82, 2.40, 0.0));
  EXPECT_TRUE(map.isLocalOccupied(index));
}
