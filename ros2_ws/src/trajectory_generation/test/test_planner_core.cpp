#include "trajectory_generation/planner_core.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <filesystem>

namespace {

class PlannerCoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_ = std::filesystem::temp_directory_path() / "trajectory_generation_core_test";
    std::filesystem::create_directories(directory_);
    occupancy_ = cv::Mat::zeros(80, 80, CV_8UC1);
    // A vertical wall with a deterministic central passage.
    cv::line(occupancy_, cv::Point(40, 0), cv::Point(40, 30), cv::Scalar(255), 1);
    cv::line(occupancy_, cv::Point(40, 49), cv::Point(40, 79), cv::Scalar(255), 1);
    write_maps();
    trajectory_generation::PlannerConfig config;
    config.resolution = 0.1;
    config.lower_x = 0.0;
    config.lower_y = 0.0;
    config.robot_radius = 0.0;
    config.desired_speed = 1.0;
    config.expected_map_width = 80;
    config.expected_map_height = 80;
    config.random_seed = 7;
    planner_.configure(config);
    planner_.load_maps(occupancy_path_.string(), bev_path_.string(), topo_path_.string());
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  void write_maps() {
    occupancy_path_ = directory_ / "occ.png";
    bev_path_ = directory_ / "bev.png";
    topo_path_ = directory_ / "topo.png";
    ASSERT_TRUE(cv::imwrite(occupancy_path_.string(), occupancy_));
    ASSERT_TRUE(cv::imwrite(bev_path_.string(), cv::Mat::zeros(80, 80, CV_8UC1)));
    ASSERT_TRUE(cv::imwrite(topo_path_.string(), cv::Mat::zeros(80, 80, CV_8UC1)));
  }

  trajectory_generation::PlannerCore planner_;
  cv::Mat occupancy_;
  std::filesystem::path directory_, occupancy_path_, bev_path_, topo_path_;
};

TEST_F(PlannerCoreTest, PlansAroundObstacleAndProducesCubicSegments) {
  ASSERT_TRUE(planner_.is_ready());
  const auto result = planner_.plan({1.0, 4.0, 0.0}, {7.0, 4.0, 0.0});
  ASSERT_GE(result.raw_path.size(), 2U);
  ASSERT_GE(result.smoothed_path.size(), 2U);
  ASSERT_FALSE(planner_.path_collides(result.smoothed_path));
  ASSERT_FALSE(result.trajectory.duration.empty());
  EXPECT_EQ(result.trajectory.coef_x.size(), 4 * result.trajectory.duration.size());
  EXPECT_EQ(result.trajectory.coef_y.size(), 4 * result.trajectory.duration.size());
  for (const auto duration : result.trajectory.duration) EXPECT_GT(duration, 0.0F);
  for (const auto coefficient : result.trajectory.coef_x) EXPECT_TRUE(std::isfinite(coefficient));
  for (const auto coefficient : result.trajectory.coef_y) EXPECT_TRUE(std::isfinite(coefficient));
  EXPECT_NEAR(result.smoothed_path.front().x(), 1.0, 0.11);
  EXPECT_NEAR(result.smoothed_path.back().x(), 7.0, 0.11);
}

TEST_F(PlannerCoreTest, MovesOccupiedGoalToNearestFreeCell) {
  const auto result = planner_.plan({1.0, 4.0, 0.0}, {4.0, 1.0, 0.0});
  ASSERT_GE(result.smoothed_path.size(), 2U);
  EXPECT_GT((result.smoothed_path.back() - Eigen::Vector3d(4.0, 1.0, 0.0)).head<2>().norm(), 0.01);
  EXPECT_FALSE(planner_.path_collides(result.smoothed_path));
}

TEST_F(PlannerCoreTest, ReturnsEmptyForNoRouteAndRejectsInvalidConfiguration) {
  occupancy_.setTo(cv::Scalar(0));
  cv::line(occupancy_, cv::Point(40, 0), cv::Point(40, 79), cv::Scalar(255), 1);
  write_maps();
  planner_.load_maps(occupancy_path_.string(), bev_path_.string(), topo_path_.string());
  const auto no_route = planner_.plan({1.0, 4.0, 0.0}, {7.0, 4.0, 0.0});
  EXPECT_TRUE(no_route.raw_path.empty());
  trajectory_generation::PlannerCore invalid;
  auto config = trajectory_generation::PlannerConfig{};
  config.resolution = 0.0;
  EXPECT_THROW(invalid.configure(config), std::invalid_argument);
}

}  // namespace
