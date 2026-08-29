#include <gtest/gtest.h>
#include <cmath>
#include <ocs2_core/PreComputation.h>
#include <ocs2_sentry/dynamics/SentryRobotDynamtics.h>

TEST(SentryDynamics, flow_map_and_jacobian_match_contract) {
  SentrySystemDynamics dynamics;
  ocs2::vector_t x(4); x << 1.0, 2.0, 3.0, M_PI / 2.0;
  ocs2::vector_t u(2); u << -0.4, 0.5;
  ocs2::PreComputation preComputation;
  const auto flow = dynamics.computeFlowMap(0.0, x, u, preComputation);
  ASSERT_NEAR(flow(0), 0.0, 1e-12);
  ASSERT_NEAR(flow(1), 3.0, 1e-12);
  ASSERT_NEAR(flow(2), -0.4, 1e-12);
  ASSERT_NEAR(flow(3), 0.5, 1e-12);
  const auto linearization = dynamics.linearApproximation(0.0, x, u, preComputation);
  EXPECT_NEAR(linearization.dfdx(0, 2), 0.0, 1e-12);
  EXPECT_NEAR(linearization.dfdx(0, 3), -3.0, 1e-12);
  EXPECT_NEAR(linearization.dfdx(1, 2), 1.0, 1e-12);
  EXPECT_NEAR(linearization.dfdx(1, 3), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(linearization.dfdu(2, 0), 1.0);
  EXPECT_DOUBLE_EQ(linearization.dfdu(3, 1), 1.0);
}
