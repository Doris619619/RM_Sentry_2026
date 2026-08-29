#include <gtest/gtest.h>
#include "trajectory_tracking/tracking_semantics.hpp"

namespace {
using trajectory_tracking::HeadingState;
using trajectory_tracking::observationHeading;
using trajectory_tracking::updateVelocityHeading;

TEST(TrackingHeading, StationaryUsesReferenceNotYaw) {
  const Eigen::Vector2d position(0.0, 0.0), target(5.0, 0.0);
  const auto state = updateVelocityHeading(0.0, 0.0, position, target, true, {});
  EXPECT_NEAR(state.heading, 0.0, 1e-12);  // yaw=pi/2 is deliberately not an input.
  EXPECT_NEAR(observationHeading(state, 0.0, true, 0.0), 0.0, 1e-12);
}

TEST(TrackingHeading, LowSpeedUsesReferenceNotYaw) {
  const Eigen::Vector2d position(0.0, 0.0), target(5.0, 0.0);
  const auto state = updateVelocityHeading(0.1, 0.0, position, target, true, {});
  EXPECT_NEAR(observationHeading(state, 0.1, true, 0.0), 0.0, 1e-12);
}

TEST(TrackingHeading, NormalSpeedUsesMapVelocityHeading) {
  const Eigen::Vector2d position(0.0, 0.0), target(5.0, 0.0);
  const auto state = updateVelocityHeading(0.0, 0.5, position, target, true, {});
  EXPECT_NEAR(observationHeading(state, 0.5, true, 0.0), M_PI_2, 1e-12);
}

TEST(TrackingHeading, StationaryLowSpeedAndNormalTransitionIsContinuous) {
  const Eigen::Vector2d position(0.0, 0.0), target(5.0, 0.0);
  auto state = updateVelocityHeading(0.0, 0.0, position, target, true, {});
  const double stationary = observationHeading(state, 0.0, true, 0.0);
  state = updateVelocityHeading(0.1, 0.0, position, target, true, state);
  const double low_speed = observationHeading(state, 0.1, true, 0.0);
  state = updateVelocityHeading(0.5, 0.0, position, target, true, state);
  const double normal = observationHeading(state, 0.5, true, 0.0);
  EXPECT_NEAR(stationary, 0.0, 1e-12);
  EXPECT_NEAR(low_speed, 0.0, 1e-12);
  EXPECT_NEAR(normal, 0.0, 1e-12);
}

TEST(TrackingHeading, MissingReferenceUsesTargetThenHistoryFallback) {
  const Eigen::Vector2d position(0.0, 0.0), target(0.0, 5.0);
  auto state = updateVelocityHeading(0.0, 0.0, position, target, true, {});
  EXPECT_NEAR(observationHeading(state, 0.0, false, 0.0), 0.0, 1e-12);
  state = updateVelocityHeading(0.0, 0.0, position, target, true, state);
  EXPECT_NEAR(observationHeading(state, 0.0, false, 0.0), M_PI_2, 1e-12);
}

TEST(TrackingArrival, OnlyExplicitPositionConditionArrives) {
  const Eigen::Vector2d target(1.0, 1.0);
  EXPECT_TRUE(trajectory_tracking::hasArrived(Eigen::Vector2d(1.1, 1.1), target, 0));
  EXPECT_FALSE(trajectory_tracking::hasArrived(Eigen::Vector2d(0.0, 0.0), target, 0));
  EXPECT_FALSE(trajectory_tracking::hasArrived(Eigen::Vector2d(1.0, 1.0), target, 8));
}
}  // namespace
