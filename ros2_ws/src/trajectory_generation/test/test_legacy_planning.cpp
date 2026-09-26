// Verify legacy map planning and polynomial continuity against exported segment durations.
#include "trajectory_generation/plan_manager.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

// Build production-map parameters for isolated legacy planner tests.
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
  const auto position = [](const Eigen::MatrixXd& coefficients, Eigen::Index segment, double time) {
    return ((coefficients(segment, 0) * time + coefficients(segment, 1)) * time +
            coefficients(segment, 2)) * time + coefficients(segment, 3);
  };
  const auto velocity = [](const Eigen::MatrixXd& coefficients, Eigen::Index segment, double time) {
    return (3.0 * coefficients(segment, 0) * time + 2.0 * coefficients(segment, 1)) * time +
           coefficients(segment, 2);
  };
  const auto& durations = manager.reference_path->m_trapezoidal_time;
  for (Eigen::Index segment = 0; segment + 1 < static_cast<Eigen::Index>(durations.size()); ++segment) {
    const double duration = durations[static_cast<std::size_t>(segment)];
    EXPECT_NEAR(position(manager.reference_path->m_polyMatrix_x, segment, duration),
                position(manager.reference_path->m_polyMatrix_x, segment + 1, 0.0), 1e-6);
    EXPECT_NEAR(position(manager.reference_path->m_polyMatrix_y, segment, duration),
                position(manager.reference_path->m_polyMatrix_y, segment + 1, 0.0), 1e-6);
    EXPECT_NEAR(velocity(manager.reference_path->m_polyMatrix_x, segment, duration),
                velocity(manager.reference_path->m_polyMatrix_x, segment + 1, 0.0), 1e-5);
    EXPECT_NEAR(velocity(manager.reference_path->m_polyMatrix_y, segment, duration),
                velocity(manager.reference_path->m_polyMatrix_y, segment + 1, 0.0), 1e-5);
  }
  for (const auto& point : manager.final_path) {
    EXPECT_FALSE(manager.global_map->isOccupied(
        manager.global_map->coord2gridIndex(Eigen::Vector3d(point.x(), point.y(), 0.0)), false));
  }
}

TEST(LegacyPlanningTest, ClampsCoordinatesAndRepairsOccupiedGoalAndDynamicObstacle) {
  auto parameters = make_parameters();
  planner_manager manager;
  manager.init(parameters);

  const auto below = manager.global_map->coord2gridIndex(Eigen::Vector3d(-100.0, -100.0, -10.0));
  const auto above = manager.global_map->coord2gridIndex(Eigen::Vector3d(100.0, 100.0, 10.0));
  EXPECT_EQ(below.x(), 0);
  EXPECT_EQ(below.y(), 0);
  EXPECT_EQ(below.z(), 0);
  EXPECT_EQ(above.x(), manager.global_map->GLX_SIZE - 1);
  EXPECT_EQ(above.y(), manager.global_map->GLY_SIZE - 1);
  EXPECT_EQ(above.z(), manager.global_map->GLZ_SIZE - 1);
  EXPECT_TRUE(manager.global_map->isOccupied(-1, 0, 0, false));
  EXPECT_TRUE(manager.global_map->isOccupied(manager.global_map->GLX_SIZE, 0, 0, false));

  Eigen::Vector3i occupied;
  bool found_occupied = false;
  for (int x = 0; x < manager.global_map->GLX_SIZE && !found_occupied; ++x) {
    for (int y = 0; y < manager.global_map->GLY_SIZE; ++y) {
      if (manager.global_map->isStaticOccupied(x, y, false)) {
        occupied = Eigen::Vector3i(x, y, 0);
        found_occupied = true;
        break;
      }
    }
  }
  ASSERT_TRUE(found_occupied);
  Eigen::Vector3i replacement;
  ASSERT_TRUE(manager.astar_path_finder->findNeighPoint(occupied, replacement, 2));
  EXPECT_FALSE(manager.global_map->isOccupied(replacement, false));

  const auto dynamic_index = manager.global_map->coord2gridIndex(Eigen::Vector3d(-3.50, 1.45, 0.0));
  const float dynamic_z = static_cast<float>(manager.global_map->getHeight(dynamic_index.x(), dynamic_index.y()) + 0.10);
  pcl::PointCloud<pcl::PointXYZ> dynamic;
  dynamic.push_back(pcl::PointXYZ(-3.50F, 1.45F, dynamic_z));
  manager.global_map->localPointCloudToObstacle(dynamic, true, Eigen::Vector3d(-3.82, 2.40, 0.0));
  EXPECT_TRUE(manager.global_map->isLocalOccupied(dynamic_index));
}


// Evaluate derivatives of the published descending-power cubic for continuity checks.
double cubicDerivative(const Eigen::MatrixXd& matrix, int segment, double time, int order) {
  if (order == 0) return ((matrix(segment,0)*time+matrix(segment,1))*time+matrix(segment,2))*time+matrix(segment,3);
  if (order == 1) return (3*matrix(segment,0)*time+2*matrix(segment,1))*time+matrix(segment,2);
  return 6*matrix(segment,0)*time+2*matrix(segment,1);
}

// Check exact knots, C1/C2 joins and stopped endpoints using the final exported durations.
void expectContinuous(const Refenecesmooth& reference, const std::vector<Eigen::Vector2d>& path, const Eigen::Vector2d& start_velocity=Eigen::Vector2d::Zero()) {
  ASSERT_EQ(reference.m_trapezoidal_time.size(), path.size()-1);
  for (const auto* matrix : {&reference.m_polyMatrix_x, &reference.m_polyMatrix_y}) {
    const int axis = matrix == &reference.m_polyMatrix_x ? 0 : 1;
    for (int i=0; i<static_cast<int>(reference.m_trapezoidal_time.size()); ++i) {
      const double t=reference.m_trapezoidal_time[i];
      for (double endpoint : {0.0,t}) {
        const double ax=cubicDerivative(reference.m_polyMatrix_x,i,endpoint,2);
        const double ay=cubicDerivative(reference.m_polyMatrix_y,i,endpoint,2);
        EXPECT_LE(std::hypot(ax,ay),reference.max_accleration+1e-7);
      }
      EXPECT_NEAR(cubicDerivative(*matrix,i,0,0),path[i][axis],1e-7);
      EXPECT_NEAR(cubicDerivative(*matrix,i,t,0),path[i+1][axis],1e-7);
      if (i+1<static_cast<int>(reference.m_trapezoidal_time.size()))
        for (int derivative=0; derivative<=2; ++derivative)
          EXPECT_NEAR(cubicDerivative(*matrix,i,t,derivative),cubicDerivative(*matrix,i+1,0,derivative),1e-6);
    }
    EXPECT_NEAR(cubicDerivative(*matrix,0,0,1),start_velocity[axis],1e-7);
    EXPECT_NEAR(cubicDerivative(*matrix,reference.m_trapezoidal_time.size()-1,reference.m_trapezoidal_time.back(),1),0,1e-7);
  }
}

// Force every feasibility iteration to change time, reproducing the last-iteration stale coefficients.
TEST(ReferenceTimingTest, ExternalDurationsStayConsistentAfterIterationLimit) {
  Refenecesmooth reference;
  std::vector<Eigen::Vector2d> path{{0,0},{1,1},{2,0},{3,.5}};
  reference.setGlobalPath(Eigen::Vector3d::Zero(),path,.1,2.,false);
  std::vector<double> times{.1,.1,.1};
  std::vector<Eigen::Vector3d> points;
  reference.getRefTrajectory(points,times);
  EXPECT_GT(reference.m_trapezoidal_time[0],times[0]);
  expectContinuous(reference,path);
}

// Exercise the legacy allocator with an intentionally strict acceleration limit.
TEST(ReferenceTimingTest, FallbackDurationsStayConsistentAfterIterationLimit) {
  auto parameters=make_parameters();planner_manager manager;manager.init(parameters);
  auto& reference=*manager.reference_path;
  std::vector<Eigen::Vector2d> path{{-.919,-4.454},{-1.419,-4.154},{-1.919,-3.804}};
  reference.setGlobalPath(Eigen::Vector3d::Zero(),path,.1,2.,false);
  std::vector<double> times;std::vector<Eigen::Vector3d> points;
  reference.getRefTrajectory(points,times);
  expectContinuous(reference,path);
}

// Reproduce stale coefficients when an otherwise feasible duration exceeds the old 30-second cap.
TEST(ReferenceTimingTest, LongTrajectoryKeepsEndpointsAndContinuity) {
  Refenecesmooth reference;
  std::vector<Eigen::Vector2d> path{{0,0},{1,1},{2,0}};
  reference.setGlobalPath(Eigen::Vector3d::Zero(),path,4.,2.,false);
  std::vector<double> times{20.,20.};std::vector<Eigen::Vector3d> points;
  reference.getRefTrajectory(points,times);
  expectContinuous(reference,path);
  EXPECT_LE(points.size(),600U);
  EXPECT_DOUBLE_EQ(reference.m_trapezoidal_time[0]+reference.m_trapezoidal_time[1],40.);
  EXPECT_NEAR((points.back().head<2>()-path.back()).norm(),0.,1e-7);
  reference.getRefVel();EXPECT_EQ(reference.reference_velocity.size(),points.size());
}

// A one-segment path has no adjacent segment for feasibility-time extension.
TEST(ReferenceTimingTest, SingleSegmentDoesNotAccessAbsentNeighbor) {
  Refenecesmooth reference;
  std::vector<Eigen::Vector2d> path{{0,0},{1,0}};
  reference.setGlobalPath(Eigen::Vector3d::Zero(),path,.1,2.,false);
  std::vector<double> times{.1};std::vector<Eigen::Vector3d> points;
  reference.getRefTrajectory(points,times);
  expectContinuous(reference,path);
}


// Invalid or nonconvergent requests must erase stale coefficients instead of exporting an unsafe path.
TEST(ReferenceTimingTest, RejectsInvalidOrUnconvergedDurations) {
  Refenecesmooth reference;
  std::vector<Eigen::Vector2d> path{{0,0},{1,0}};
  std::vector<Eigen::Vector3d> points;
  std::vector<double> times{1.};
  reference.setGlobalPath(Eigen::Vector3d::Zero(),path,4.,2.,false);
  reference.getRefTrajectory(points,times);
  ASSERT_FALSE(points.empty());
  times[0]=0.;
  reference.getRefTrajectory(points,times);
  EXPECT_TRUE(points.empty());EXPECT_TRUE(reference.m_trapezoidal_time.empty());
  EXPECT_EQ(reference.m_polyMatrix_x.rows(),0);
  times[0]=.1;
  reference.setGlobalPath(Eigen::Vector3d::Zero(),path,1e-20,2.,false);
  reference.getRefTrajectory(points,times);
  EXPECT_TRUE(points.empty());EXPECT_TRUE(reference.m_trapezoidal_time.empty());
  reference.getRefVel();EXPECT_TRUE(reference.reference_velocity.empty());
}

// Duration stretching must preserve measured initial velocity rather than slowing that boundary state.
TEST(ReferenceTimingTest, PreservesNonzeroInitialVelocityWhileConverging) {
  Refenecesmooth reference;
  std::vector<Eigen::Vector2d> path{{0,0},{1,1},{2,0}};
  const Eigen::Vector2d initial(.2,-.1);
  reference.setGlobalPath(Eigen::Vector3d(initial.x(),initial.y(),0),path,4.,2.,false);
  std::vector<double> times{.1,.1};std::vector<Eigen::Vector3d> points;
  reference.getRefTrajectory(points,times);
  ASSERT_FALSE(points.empty());
  expectContinuous(reference,path,initial);
}

// Live occupied cells must obstruct both PRM visibility and path shortcut pruning.
TEST(LegacyPlanningTest, DynamicOccupancyBlocksVisibilityAndClears) {
  auto parameters = make_parameters();
  planner_manager manager;
  manager.init(parameters);
  Eigen::Vector3d a(-3.4,-2.7,0.0), b(-4.3,-2.1,0.0);
  a.z() = manager.global_map->getHeight(manager.global_map->coord2gridIndex(a).x(), manager.global_map->coord2gridIndex(a).y());
  b.z() = a.z();
  Eigen::Vector3d collision;
  ASSERT_TRUE(manager.topo_prm->lineVisib(a,b,0.05,collision));
  auto index = manager.global_map->coord2gridIndex((a+b)*0.5);
  auto& cell = manager.global_map->l_data[index.x()*manager.global_map->GLY_SIZE+index.y()];
  cell = 10;
  EXPECT_FALSE(manager.topo_prm->lineVisib(a,b,0.05,collision));
  cell = 0;
  EXPECT_TRUE(manager.topo_prm->lineVisib(a,b,0.05,collision));
}

}  // namespace

// Reproduce both directions of the autonomous route with its larger planning clearance.
TEST(LegacyPlanningTest, AutonomousRouteIsConnectedInBothDirections) {
  auto parameters = make_parameters();
  parameters.setParam("trajectory_generator/robot_radius", 0.45);
  planner_manager manager;
  manager.init(parameters);
  const Eigen::Vector3d a(-0.919,-4.454,0), b(-3.219,2.146,0);
  for (int reverse=0; reverse<2; ++reverse) {
    manager.topo_prm->setRandomSeed(7);
    const auto start=reverse ? b : a, goal=reverse ? a : b;
    manager.global_map->odom_position=start;
    EXPECT_FALSE(manager.global_map->isStaticOccupied(manager.global_map->coord2gridIndex(start),false));
    EXPECT_FALSE(manager.global_map->isStaticOccupied(manager.global_map->coord2gridIndex(goal),false));
    manager.topo_prm->createGraph(start,goal);
    EXPECT_GE(manager.topo_prm->min_path.size(),2U) << "reverse=" << reverse;
  }
}
