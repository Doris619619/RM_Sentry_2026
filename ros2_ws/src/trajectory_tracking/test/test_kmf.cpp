#include <gtest/gtest.h>
#include <Eigen/Core>
#include "KMF.h"

TEST(Kmf, FixedMeasurementSequence) {
  KMF filter;
  filter.initParam(0.1, 0.01, 0.1, true);
  filter.setState(Eigen::Vector2d::Zero());
  filter.predictUpdate(0.1);
  filter.measureUpdate((Eigen::Vector2d() << 1.0, 0.0).finished());
  Eigen::Vector2d output;
  filter.getResults(output);
  EXPECT_NEAR(output.x(), 0.5027126799306487, 1e-12);
  EXPECT_NEAR(output.y(), 0.04480065946570738, 1e-12);
  filter.predictUpdate(0.1, 0.5);
  filter.measureUpdate((Eigen::Vector2d() << 1.0, 0.0).finished());
  filter.getResults(output);
  EXPECT_TRUE(output.allFinite());
  EXPECT_GT(output.x(), 0.5);
  EXPECT_LT(output.x(), 1.0);
}
