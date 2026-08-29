#include <gtest/gtest.h>
#include "trajectory_tracking/local_planner.hpp"

TEST(LocalPlanner, SamplesRos1CubicOrderAndClampsLead) {
  trajectory_tracking::Polynomial polynomial;
  polynomial.duration = {4.0};
  polynomial.x = {0.0, 0.0, 1.0, 0.0};
  polynomial.y = {0.0, 0.0, 0.0, 0.0};
  polynomial.valid = true;
  trajectory_tracking::LocalPlanner planner;
  ASSERT_TRUE(planner.setTrajectory(polynomial));
  trajectory_tracking::LocalPlanner::Reference reference;
  ASSERT_TRUE(planner.makeFightReference(Eigen::Vector3d(0.0, 0.0, 0.0), 3.0, 0.0, reference));
  ASSERT_EQ(reference.states.size(), 20u);
  EXPECT_NEAR(reference.states.front()(0), 0.5, 1e-9);  // 0.5 s maximum lead
  EXPECT_NEAR(reference.states.front()(2), 1.0, 1e-9);
}

TEST(LocalPlanner, Mode8NearGoalUsesCircleAndRequestsRedecision) {
  trajectory_tracking::Polynomial polynomial;
  polynomial.duration = {1.0};
  polynomial.x = {0.0, 0.0, 0.0, 2.0};
  polynomial.y = {0.0, 0.0, 0.0, 3.0};
  polynomial.motion_mode = 8;
  polynomial.valid = true;
  trajectory_tracking::LocalPlanner planner;
  ASSERT_TRUE(planner.setTrajectory(polynomial));
  trajectory_tracking::LocalPlanner::Reference reference;
  ASSERT_TRUE(planner.makeFightReference(Eigen::Vector3d(2.5, 3.0, 0.0), 0.0, 0.0, reference));
  EXPECT_TRUE(reference.redecision);
  EXPECT_NEAR((reference.states.front().head<2>() - Eigen::Vector2d(2.0, 3.0)).norm(), 0.8, 1e-9);
  EXPECT_NEAR(reference.states.front()(2), 1.3, 1e-9);
}
