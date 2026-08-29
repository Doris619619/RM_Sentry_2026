#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace trajectory_tracking {

struct Polynomial {
  std::vector<double> duration;
  std::vector<double> x;
  std::vector<double> y;
  uint8_t motion_mode{0};
  bool valid{false};
};

/** trajectory_generation/TrajectoryPoly uses [cubic, quadratic, linear, constant], exactly as ROS1 LocalPlanner. */
inline bool validatePolynomial(const Polynomial& polynomial) {
  const std::size_t pieces = polynomial.duration.size();
  if (pieces == 0 || polynomial.x.size() != 4 * pieces || polynomial.y.size() != 4 * pieces) return false;
  for (std::size_t piece = 0; piece < pieces; ++piece) {
    if (!(polynomial.duration[piece] > 0.0) || !std::isfinite(polynomial.duration[piece])) return false;
    for (std::size_t coefficient = 0; coefficient < 4; ++coefficient) {
      if (!std::isfinite(polynomial.x[4 * piece + coefficient]) ||
          !std::isfinite(polynomial.y[4 * piece + coefficient])) return false;
    }
  }
  return true;
}

inline bool samplePolynomial(const Polynomial& polynomial, double elapsed, Eigen::Vector4d& state) {
  if (!polynomial.valid || !std::isfinite(elapsed)) return false;
  std::size_t piece = 0;
  double local = std::max(0.0, elapsed);
  while (piece + 1 < polynomial.duration.size() && local > polynomial.duration[piece]) local -= polynomial.duration[piece++];
  local = std::min(local, polynomial.duration[piece]);
  const auto eval = [piece, local](const std::vector<double>& coefficients) {
    const std::size_t i = 4 * piece;
    return coefficients[i] * local * local * local + coefficients[i + 1] * local * local +
           coefficients[i + 2] * local + coefficients[i + 3];
  };
  const auto derivative = [piece, local](const std::vector<double>& coefficients) {
    const std::size_t i = 4 * piece;
    return 3.0 * coefficients[i] * local * local + 2.0 * coefficients[i + 1] * local + coefficients[i + 2];
  };
  const double vx = derivative(polynomial.x);
  const double vy = derivative(polynomial.y);
  state << eval(polynomial.x), eval(polynomial.y), std::hypot(vx, vy), std::atan2(vy, vx);
  return state.allFinite();
}

}  // namespace trajectory_tracking
