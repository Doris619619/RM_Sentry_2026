#include "trajectory_tracking/local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_tracking {
namespace {
double unwrapNear(double value, double reference) {
  while (value - reference > M_PI) value -= 2.0 * M_PI;
  while (value - reference < -M_PI) value += 2.0 * M_PI;
  return value;
}
}

bool LocalPlanner::setTrajectory(const Polynomial& trajectory) {
  if (!validatePolynomial(trajectory)) return false;
  trajectory_ = trajectory;
  totalDuration_ = 0.0;
  for (double duration : trajectory_.duration) totalDuration_ += duration;
  path_.clear();
  // This is the ROS1 getRefTrajectory/getRefVel sampling contract, including
  // the final endpoint. The solver reference is then linearly interpolated.
  for (double time = 0.0; time <= totalDuration_ + 1e-9; time += kDt) {
    Eigen::Vector4d state;
    if (!samplePolynomial(trajectory_, std::min(time, totalDuration_), state)) return false;
    path_.push_back(state);
  }
  Eigen::Vector4d endpoint;
  if (!samplePolynomial(trajectory_, totalDuration_, endpoint)) return false;
  if (path_.empty() || (path_.back().head<2>() - endpoint.head<2>()).norm() > 1e-9) path_.push_back(endpoint);
  target_ = endpoint.head<2>();
  trackingLow_.clear();
  return path_.size() >= 2;
}

void LocalPlanner::reset() { trajectory_ = Polynomial{}; path_.clear(); totalDuration_ = 0.0; trackingLow_.clear(); }

bool LocalPlanner::makeFightReference(const Eigen::Vector3d& position, double elapsed,
                                      double robot_yaw, Reference& reference) {
  if (!trajectory_.valid || path_.size() < 2 || !std::isfinite(elapsed)) return false;
  double nearestDistance = std::numeric_limits<double>::infinity();
  std::size_t nearest = 0;
  for (std::size_t i = 0; i < path_.size(); ++i) {
    const double distance = (position.head<2>() - path_[i].head<2>()).norm();
    if (distance < nearestDistance) { nearestDistance = distance; nearest = i; }
  }
  // ROS1 LocalPlanner MAX_LEAD_TIME=0.5: hold the tracking clock rather than
  // allowing the reference to run away from the physical robot.
  elapsed = std::min(elapsed, nearest * kDt + 0.5);
  trackingLow_.push_back(nearestDistance > 0.8);
  if (trackingLow_.size() > 10) trackingLow_.erase(trackingLow_.begin());
  reference = Reference{};
  reference.off_course = std::count(trackingLow_.begin(), trackingLow_.end(), true) > 4;
  const bool evade = trajectory_.motion_mode == 8 && (position.head<2>() - target_).norm() < 1.2;
  double previousPhi = robot_yaw;
  for (int i = 0; i < kHorizon; ++i) {
    Eigen::Vector4d state;
    if (evade) {
      const double base = std::atan2(position.y() - target_.y(), position.x() - target_.x());
      const double theta = base + (i + 1) * 0.1625;
      state << target_.x() + 0.8 * std::cos(theta), target_.y() + 0.8 * std::sin(theta),
          1.3, theta + M_PI_2;
      reference.redecision = true;
    } else {
      const double sampleTime = elapsed + i * kDt;
      const double sampleIndex = sampleTime / kDt;
      const std::size_t lo = static_cast<std::size_t>(std::max(0.0, std::floor(sampleIndex)));
      if (lo + 1 >= path_.size()) {
        state = path_.back();
        state(2) = 0.0;  // ROS1 holds the terminal pose with zero velocity.
      } else {
        const double alpha = sampleIndex - std::floor(sampleIndex);
        state = (1.0 - alpha) * path_[lo] + alpha * path_[lo + 1];
      }
      // The polynomial sampler already supplies analytic speed and heading.
      state(3) = unwrapNear(state(3), previousPhi);
      const double distanceToGoal = (state.head<2>() - target_).norm();
      if (distanceToGoal < 1.5) state(2) *= std::max(0.05, distanceToGoal / 1.5);
    }
    state(3) = unwrapNear(state(3), previousPhi);
    previousPhi = state(3);
    if (!state.allFinite()) return false;
    reference.times.push_back(i * kDt);
    reference.states.push_back(state);
  }
  return true;
}

}  // namespace trajectory_tracking
