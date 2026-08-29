#include <gtest/gtest.h>

#include <trajectory_tracking/trajectory_poly.hpp>

namespace {
trajectory_tracking::Polynomial makePolynomial() {
  trajectory_tracking::Polynomial polynomial;
  polynomial.duration = {2.0, 3.0};
  // ROS1 order is cubic, quadratic, linear, constant.
  polynomial.x = {1.0, 2.0, 3.0, 4.0, 0.0, 0.0, -2.0, 10.0};
  polynomial.y = {0.0, 0.0, 4.0, -1.0, 0.0, 0.0, 1.0, 7.0};
  polynomial.valid = trajectory_tracking::validatePolynomial(polynomial);
  return polynomial;
}
}  // namespace

TEST(TrajectoryPoly, preserves_ros1_cubic_to_constant_order) {
  const auto polynomial = makePolynomial();
  ASSERT_TRUE(polynomial.valid);
  Eigen::Vector4d state;
  ASSERT_TRUE(trajectory_tracking::samplePolynomial(polynomial, 2.0, state));
  EXPECT_DOUBLE_EQ(state.x(), 26.0);
  EXPECT_DOUBLE_EQ(state.y(), 7.0);
  EXPECT_DOUBLE_EQ(state.z(), std::hypot(23.0, 4.0));
  EXPECT_DOUBLE_EQ(state.w(), std::atan2(4.0, 23.0));
}

TEST(TrajectoryPoly, carries_over_to_next_piece_with_correct_local_time) {
  const auto polynomial = makePolynomial();
  Eigen::Vector4d state;
  ASSERT_TRUE(trajectory_tracking::samplePolynomial(polynomial, 2.5, state));
  EXPECT_DOUBLE_EQ(state.x(), 9.0);
  EXPECT_DOUBLE_EQ(state.y(), 7.5);
  EXPECT_DOUBLE_EQ(state.z(), std::hypot(2.0, 1.0));
  EXPECT_DOUBLE_EQ(state.w(), std::atan2(1.0, -2.0));
}

TEST(TrajectoryPoly, rejects_malformed_and_non_finite_input_before_sampling) {
  auto polynomial = makePolynomial();
  polynomial.x.pop_back();
  EXPECT_FALSE(trajectory_tracking::validatePolynomial(polynomial));
  polynomial = makePolynomial();
  polynomial.duration[0] = 0.0;
  EXPECT_FALSE(trajectory_tracking::validatePolynomial(polynomial));
}
