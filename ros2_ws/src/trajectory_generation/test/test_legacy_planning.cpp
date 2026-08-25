#include "trajectory_generation/plan_manager.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

ros::NodeHandle make_parameters() {
  ros::NodeHandle parameters;
  const std::string maps = TEST_LEGACY_MAP_DIR;
  parameters.setParam("trajectory_generator/occ_file_path", maps + "/occfinal.png");
  parameters.setParam("trajectory_generator/bev_file_path", maps + "/bevfinal.png");
  parameters.setParam("trajectory_generator/distance_map_file_path", maps + "/occtopo.png");
  parameters.setParam("trajectory_generator/map_resolution", 0.05);
  parameters.setParam("trajectory_generator/map_x_size", 20.0);
  parameters.setParam("trajectory_generator/map_y_size", 20.0);
  parameters.setParam("trajectory_generator/map_z_size", 2.0);
  parameters.setParam("trajectory_generator/map_lower_point_x", -13.394);
  parameters.setParam("trajectory_generator/map_lower_point_y", -12.079);
  parameters.setParam("trajectory_generator/map_lower_point_z", 0.0);
  parameters.setParam("trajectory_generator/robot_radius", 0.35);
  parameters.setParam("trajectory_generator/robot_radius_dash", 0.35);
  parameters.setParam("trajectory_generator/reference_desire_speed", 2.0);
  parameters.setParam("trajectory_generator/reference_desire_speedxtl", 2.4);
  parameters.setParam("trajectory_generator/reference_a_max", 4.0);
  parameters.setParam("trajectory_generator/search_height_min", -0.05);
  parameters.setParam("trajectory_generator/search_height_max", 1.2);
  parameters.setParam("trajectory_generator/search_radius", 6.0);
  parameters.setParam("trajectory_generator/height_bias", 0.015294117853045464);
  parameters.setParam("trajectory_generator/height_interval", 1.5);
  parameters.setParam("trajectory_generator/height_threshold", 0.08);
  parameters.setParam("trajectory_generator/height_sencond_high_threshold", 0.2);
  return parameters;
}

TEST(LegacyPlanningTest, ConvertsMapCoordinatesAndBuildsTopoPath) {
  auto parameters = make_parameters();
  planner_manager manager;
  manager.init(parameters);
  manager.topo_prm->setRandomSeed(7);

  const Eigen::Vector3d start(-3.82, 2.40, 0.0);
  const Eigen::Vector3d goal(-1.35, -4.20, 0.0);
  const auto index = manager.global_map->coord2gridIndex(start);
  const auto restored = manager.global_map->gridIndex2coord(index);
  EXPECT_NEAR(restored.x(), start.x(), 0.051);
  EXPECT_NEAR(restored.y(), start.y(), 0.051);
  EXPECT_FALSE(manager.global_map->isOccupied(index, false));

  ASSERT_TRUE(manager.pathFinding(start, goal, Eigen::Vector3d::Zero()));
  ASSERT_GE(manager.astar_path.size(), 2U);
  ASSERT_GE(manager.optimized_path.size(), 2U);
  ASSERT_GE(manager.final_path.size(), 2U);
  ASSERT_FALSE(manager.reference_path->m_trapezoidal_time.empty());
  EXPECT_EQ(manager.reference_path->m_polyMatrix_x.rows(),
            static_cast<Eigen::Index>(manager.reference_path->m_trapezoidal_time.size()));
  EXPECT_EQ(manager.reference_path->m_polyMatrix_y.rows(),
            static_cast<Eigen::Index>(manager.reference_path->m_trapezoidal_time.size()));
  for (const auto duration : manager.reference_path->m_trapezoidal_time) {
    EXPECT_TRUE(std::isfinite(duration));
    EXPECT_GT(duration, 0.0);
  }
}

}  // namespace
