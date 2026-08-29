#pragma once

#include <Eigen/Core>
#include <vector>
#include "trajectory_tracking/trajectory_poly.hpp"

namespace trajectory_tracking {

/** ROS2, transport-free port of the reference side of ROS1 LocalPlanner.
 * It keeps the original 0.1 s sampled trajectory, 0.5 s lead clamp and
 * mode-8 circular evade reference. Solver ownership remains in the ROS adapter.
 */
class LocalPlanner {
 public:
  struct Reference {
    std::vector<double> times;
    std::vector<Eigen::Vector4d> states;
    bool redecision{false};
    bool off_course{false};
  };

  bool setTrajectory(const Polynomial& trajectory);
  void reset();
  bool makeFightReference(const Eigen::Vector3d& position, double elapsed,
                          double robot_yaw, Reference& reference);
  const Eigen::Vector2d& target() const { return target_; }
  double totalDuration() const { return totalDuration_; }

 private:
  static constexpr double kDt = 0.1;
  static constexpr int kHorizon = 20;
  Polynomial trajectory_;
  std::vector<Eigen::Vector4d> path_;
  Eigen::Vector2d target_{Eigen::Vector2d::Zero()};
  double totalDuration_{0.0};
  std::vector<bool> trackingLow_;
};

}  // namespace trajectory_tracking
