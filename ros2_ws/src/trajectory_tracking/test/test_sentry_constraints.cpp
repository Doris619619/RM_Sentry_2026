#include <gtest/gtest.h>
#include <ocs2_core/PreComputation.h>
#include "ocs2_sentry/constraint/SentryRobotCollisionConstraint.h"
#include "ocs2_sentry/constraint/SentryRobotStateInputConstraint.h"

TEST(SentryCollisionConstraint, ValuesAndJacobianFollowStaticAndDynamicThresholds) {
  ocs2::scalar_array_t times{0.0, 0.1};
  std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> obstacles(2);
  obstacles[0].push_back({0, Eigen::Vector3d(1.0, 2.0, 0.0)});
  obstacles[0].push_back({1, Eigen::Vector3d(3.0, 2.0, 0.0)});
  auto set = std::make_shared<ObsConstraintSet>(times, obstacles);
  SentryCollisionConstraint constraint(set);
  ocs2::PreComputation precomputation;
  ocs2::vector_t state(4); state << 2.0, 2.0, 0.0, 0.0;
  const auto value = constraint.getValue(0.0, state, precomputation);
  ASSERT_EQ(value.size(), 2);
  EXPECT_NEAR(value(0), 0.96, 1e-12);
  EXPECT_NEAR(value(1), 0.84, 1e-12);
  const auto linear = constraint.getLinearApproximation(0.0, state, precomputation);
  EXPECT_NEAR(linear.dfdx(0, 0), 2.0, 1e-12);
  EXPECT_NEAR(linear.dfdx(1, 0), -2.0, 1e-12);
}

TEST(SentryStateInputConstraint, KeepsRos1SoftLimits) {
  SentryStateInputConstraint constraint;
  ocs2::PreComputation precomputation;
  ocs2::vector_t state(4); state << 0.0, 0.0, 2.5, 0.0;
  ocs2::vector_t input(2); input << 3.5, 6.0;
  const auto value = constraint.getValue(0.0, state, input, precomputation);
  ASSERT_EQ(value.size(), 6);
  EXPECT_NEAR(value(0), 0.0, 1e-12);  // a = +3.5
  EXPECT_NEAR(value(2), 0.0, 1e-12);  // w = +6.0
  EXPECT_NEAR(value(4), 0.0, 1e-12);  // v = +2.5
  EXPECT_NEAR(value(3), 12.0, 1e-12);
  EXPECT_NEAR(value(5), 5.0, 1e-12);
}
