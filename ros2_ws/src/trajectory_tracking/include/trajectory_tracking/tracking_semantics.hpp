#pragma once

#include <cmath>
#include <Eigen/Core>

namespace trajectory_tracking {

struct HeadingState {
  double heading{0.0};
  bool initialized{false};
};

inline double wrapHeading(double heading) {
  return std::remainder(heading, 2.0 * M_PI);
}

// ROS1 Fix45/46 contract: phi is world-frame velocity heading, not base/LiDAR yaw.
inline HeadingState updateVelocityHeading(double world_vx, double world_vy,
                                          const Eigen::Vector2d& position,
                                          const Eigen::Vector2d& target,
                                          bool target_available,
                                          HeadingState previous,
                                          double velocity_epsilon = 0.03) {
  const double speed = std::hypot(world_vx, world_vy);
  double raw_heading = 0.0;
  if (speed > velocity_epsilon) {
    raw_heading = std::atan2(world_vy, world_vx);
  } else if (previous.initialized && target_available) {
    const Eigen::Vector2d delta = target - position;
    raw_heading = delta.norm() > 0.3 ? std::atan2(delta.y(), delta.x()) : previous.heading;
  } else {
    // Fix46b: a stationary robot has no velocity heading; never seed it with yaw.
    raw_heading = previous.initialized ? previous.heading : 0.0;
  }

  if (!previous.initialized) return {wrapHeading(raw_heading), true};
  const double delta = std::remainder(raw_heading - previous.heading, 2.0 * M_PI);
  return {wrapHeading(previous.heading + delta), true};
}

inline double observationHeading(const HeadingState& velocity_heading, double speed,
                                 bool reference_available, double reference_phi,
                                 double reference_override_speed = 0.3) {
  // Fix46a: for an omni base at low speed, the reference direction is the valid
  // MPC phi. This also prevents a stale/yaw-derived direction from bending MPC.
  if (speed < reference_override_speed && reference_available && std::isfinite(reference_phi)) {
    return wrapHeading(reference_phi);
  }
  return velocity_heading.heading;
}

inline bool hasArrived(const Eigen::Vector2d& position, const Eigen::Vector2d& target,
                       int motion_mode, double distance_threshold = 0.3) {
  return motion_mode != 8 && (position - target).norm() < distance_threshold;
}

}  // namespace trajectory_tracking
