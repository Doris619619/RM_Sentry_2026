#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <sentry_msgs/msg/slaver_speed.hpp>
#include <sentry_msgs/msg/robot_status.hpp>
#include <sentry_msgs/msg/robots_hp.hpp>
#include <trajectory_generation/msg/trajectory_poly.hpp>
#include "ocs2_sentry/SentryRobotInterface.h"
#include "KMF.h"
#include "trajectory_tracking/trajectory_poly.hpp"
#include "trajectory_tracking/local_planner.hpp"
#include "trajectory_tracking/RM_GridMap.h"
#include "trajectory_tracking/tracking_semantics.hpp"

namespace {
double yawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
bool isFinite(double v) { return std::isfinite(v); }
}

class TrajectoryTrackingNode final : public rclcpp::Node {
 public:
  TrajectoryTrackingNode() : Node("trajectory_tracking") {
    const auto defaultTask = ament_index_cpp::get_package_share_directory("trajectory_tracking") + "/cfg/task.info";
    taskFile_ = declare_parameter<std::string>("task_file", defaultTask);
    outputMaxSpeed_ = declare_parameter<double>("output.max_speed", 2.0);
    referenceDt_ = declare_parameter<double>("reference.dt", 0.1);
    velocityEpsilon_ = declare_parameter<double>("heading.velocity_epsilon", 0.03);
    referenceOverrideSpeed_ = declare_parameter<double>("heading.reference_override_speed", 0.3);
    arrivalDistance_ = declare_parameter<double>("arrival.distance", 0.3);
    collisionHorizonSteps_ = declare_parameter<int>("collision.horizon_steps", 20);
    collisionTimeStep_ = declare_parameter<double>("collision.time_step", 0.1);
    collisionPenaltyMu_ = declare_parameter<double>("collision.penalty_mu", 20.0);
    collisionPenaltyDelta_ = declare_parameter<double>("collision.penalty_delta", 0.5);
    if (std::abs(referenceDt_ - 0.1) > 1e-12 || collisionHorizonSteps_ != 20 ||
        std::abs(collisionTimeStep_ - 0.1) > 1e-12 || velocityEpsilon_ <= 0.0 ||
        referenceOverrideSpeed_ <= 0.0 || arrivalDistance_ <= 0.0) {
      throw std::invalid_argument("tracking parameters violate the frozen ROS1 MPC contract");
    }

    robot_.init(taskFile_);
    // This is the active ROS1 LocalPlanner collision wiring: a single shared
    // obstacle set is updated before every real SQP solve, while the solver and
    // HPIPM warm start survive across odometry callbacks.
    ocs2::scalar_array_t collisionTimes(static_cast<std::size_t>(collisionHorizonSteps_));
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> collisionPoints(
      static_cast<std::size_t>(collisionHorizonSteps_));
    for (std::size_t i = 0; i < collisionTimes.size(); ++i) collisionTimes[i] = collisionTimeStep_ * i;
    robot_.obsConstraintPtr_ = std::make_shared<ObsConstraintSet>(collisionTimes, collisionPoints);
    const ocs2::RelaxedBarrierPenalty::Config collisionPenalty(collisionPenaltyMu_, collisionPenaltyDelta_);
    robot_.problem_.stateSoftConstraintPtr->add(
      "stateCollisionBounds",
      std::make_unique<ocs2::StateSoftConstraint>(
        std::make_unique<SentryCollisionConstraint>(robot_.obsConstraintPtr_),
        std::make_unique<ocs2::RelaxedBarrierPenalty>(collisionPenalty)));
    robot_.sqpSettings().hpipmSettings.hpipmMode = hpipm_mode::ROBUST;
    robot_.sqpSettings().hpipmSettings.reg_prim = 1e-8;
    solver_ = std::make_unique<ocs2::SqpMpc>(
      robot_.mpcSettings(), robot_.sqpSettings(), robot_.getOptimalControlProblem(), robot_.getInitializer());
    solver_->getSolverPtr()->setReferenceManager(robot_.getReferenceManagerPtr());

    // Reuse the ROS1 RM_GridMap algorithm with the ROS2 global-planning map contract.
    // ROS2 owns the subscriptions; the compatibility NodeHandle is only the original
    // algorithm's parameter carrier.
    const auto globalPlanningShare = ament_index_cpp::get_package_share_directory("trajectory_generation");
    const double mapResolution = declare_parameter<double>("map.resolution", 0.05);
    const double mapXSize = declare_parameter<double>("map.x_size", 20.0);
    const double mapYSize = declare_parameter<double>("map.y_size", 20.0);
    const double mapZSize = declare_parameter<double>("map.z_size", 2.0);
    const double mapLowerX = declare_parameter<double>("map.lower_x", -13.394);
    const double mapLowerY = declare_parameter<double>("map.lower_y", -12.079);
    const double mapLowerZ = declare_parameter<double>("map.lower_z", 0.0);
    const double mapRobotRadius = declare_parameter<double>("map.robot_radius", 0.35);
    const double mapSearchHeightMin = declare_parameter<double>("map.search_height_min", -0.05);
    const double mapSearchHeightMax = declare_parameter<double>("map.search_height_max", 1.2);
    const double mapSearchRadius = declare_parameter<double>("map.search_radius", 6.0);
    const double mapHeightBias = declare_parameter<double>("map.height_bias", 0.015294117853045464);
    const double mapHeightInterval = declare_parameter<double>("map.height_interval", 1.5);
    const double mapHeightThreshold = declare_parameter<double>("map.height_threshold", 0.08);
    const double mapSecondHeightThreshold = declare_parameter<double>("map.height_second_high_threshold", 0.2);
    if (mapResolution <= 0.0 || mapXSize <= 0.0 || mapYSize <= 0.0 || mapZSize <= 0.0 ||
        mapRobotRadius <= 0.0 || mapSearchRadius <= 0.0 || mapSearchHeightMin > mapSearchHeightMax ||
        mapHeightInterval <= 0.0) {
      throw std::invalid_argument("invalid shared map metadata");
    }
    ros::NodeHandle mapParameters;
    mapParameters.setParam("trajectory_generator/height_bias", mapHeightBias);
    mapParameters.setParam("trajectory_generator/height_interval", mapHeightInterval);
    mapParameters.setParam("trajectory_generator/height_threshold", mapHeightThreshold);
    mapParameters.setParam("trajectory_generator/height_sencond_high_threshold", mapSecondHeightThreshold);
    const Eigen::Vector3d mapLower(mapLowerX, mapLowerY, mapLowerZ);
    const Eigen::Vector3d mapUpper = mapLower + Eigen::Vector3d(mapXSize, mapYSize, mapZSize);
    gridMap_ = std::make_unique<TrackingGridMap>();
    gridMap_->initGridMap(
      mapParameters, globalPlanningShare + "/map/occfinal.png",
      globalPlanningShare + "/map/bevfinal.png", globalPlanningShare + "/map/occtopo.png",
      mapResolution, mapLower, mapUpper,
      static_cast<int>(std::lround(mapXSize / mapResolution)),
      static_cast<int>(std::lround(mapYSize / mapResolution)),
      static_cast<int>(std::lround(mapZSize / mapResolution)), mapRobotRadius,
      mapSearchHeightMin, mapSearchHeightMax, mapSearchRadius);
    gridMapReady_ = gridMap_->data != nullptr;
    if (!gridMapReady_) RCLCPP_ERROR(get_logger(), "RM_GridMap initialization failed; dynamic-obstacle replanning disabled");
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    trajectorySub_ = create_subscription<trajectory_generation::msg::TrajectoryPoly>(
      "/global_trajectory", rclcpp::QoS(10).reliable(),
      std::bind(&TrajectoryTrackingNode::trajectoryCallback, this, std::placeholders::_1));
    odometrySub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odometry", rclcpp::QoS(5).reliable(),
      std::bind(&TrajectoryTrackingNode::odometryCallback, this, std::placeholders::_1));
    alignedPointsSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/aligned_points", rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryTrackingNode::alignedPointsCallback, this, std::placeholders::_1));
    wheelStateSub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "/slaver/wheel_state", rclcpp::QoS(2), [this](geometry_msgs::msg::Vector3::ConstSharedPtr message) {
        if (std::abs(message->x) < 10.0) wheelSpeed_ = message->x;
        if (std::isfinite(message->y)) wheelYaw_ = message->y * M_PI / 180.0;
      });
    robotStatusSub_ = create_subscription<sentry_msgs::msg::RobotStatus>(
      "/slaver/robot_status", rclcpp::QoS(1), [this](sentry_msgs::msg::RobotStatus::ConstSharedPtr message) {
        teamIsRed_ = message->id == 7;
      });
    robotHpSub_ = create_subscription<sentry_msgs::msg::RobotsHP>(
      "/slaver/robot_HP", rclcpp::QoS(1), [this](sentry_msgs::msg::RobotsHP::ConstSharedPtr message) {
        updateHpState(*message);
      });
    speedPub_ = create_publisher<sentry_msgs::msg::SlaverSpeed>("/sentry_des_speed", rclcpp::QoS(10).reliable());
    solverStatusPub_ = create_publisher<std_msgs::msg::Bool>("/solver_status", rclcpp::QoS(10).reliable());
    replanPub_ = create_publisher<std_msgs::msg::Bool>("/replan_flag", rclcpp::QoS(10).reliable());
    redecisionPub_ = create_publisher<std_msgs::msg::Bool>("/redecide_flag", rclcpp::QoS(10).reliable());
    arrivedPub_ = create_publisher<std_msgs::msg::Bool>("/tracking/arrived", rclcpp::QoS(10).reliable());
    robotYawPub_ = create_publisher<std_msgs::msg::Float64>("/robot_cur_yaw_reg", rclcpp::QoS(10).reliable());
    predictedPathPub_ = create_publisher<nav_msgs::msg::Path>("/tracking/mpc_predicted_path", rclcpp::QoS(10).reliable());
    referencePathPub_ = create_publisher<nav_msgs::msg::Path>("/tracking/mpc_reference_path", rclcpp::QoS(10).reliable());
    candidateMarkerPub_ = create_publisher<visualization_msgs::msg::Marker>("candidate_path_vis", rclcpp::QoS(10).reliable());
    referenceMarkerPub_ = create_publisher<visualization_msgs::msg::Marker>("reference_path_vis", rclcpp::QoS(10).reliable());
    obstacleMarkerPub_ = create_publisher<visualization_msgs::msg::Marker>("obs_center_vis", rclcpp::QoS(10).reliable());
  }

 private:
  void updateHpState(const sentry_msgs::msg::RobotsHP& message) {
    mateOutpostHp_ = teamIsRed_ ? message.red_outpost_hp : message.blue_outpost_hp;
    if (mateOutpostHp_ < 1) {
      if (!isXtl_) replanNow_ = true;
      isXtl_ = true;
    } else {
      isXtl_ = false;
    }
    ++hpStableCount_;
    const int currentHp = teamIsRed_ ? message.red_sentry_hp : message.blue_sentry_hp;
    if (sentryHp_ == 0) sentryHp_ = currentHp;
    if (sentryHp_ - currentHp >= 2) {
      hpStableCount_ = 0;
      isAttacked_ = true;
    } else if (hpStableCount_ > 20 && sentryHp_ == currentHp) {
      isAttacked_ = false;
    }
    sentryHp_ = currentHp;
  }

  bool predictedInBridge(const ocs2::PrimalSolution& solution) const {
    if (!gridMapReady_) return false;
    for (const auto& state : solution.stateTrajectory_) {
      const auto index = gridMap_->coord2gridIndex(Eigen::Vector3d(state(0), state(1), 0.0));
      if (gridMap_->GridNodeMap[index.x()][index.y()]->exist_second_height) return true;
    }
    return false;
  }

  uint8_t selectMotionMode(bool arrived, bool inBridge, const trajectory_tracking::Polynomial& trajectory) {
    const double duration = std::accumulate(trajectory.duration.begin(), trajectory.duration.end(), 0.0);
    int candidate = 0;
    if (!isXtl_) {
      candidate = arrived ? 2 : (duration > 0.5 && !inBridge ? 0 : 1);
    } else if (!arrived) {
      candidate = (isAttacked_ || trajectory.motion_mode == 8 || trajectory.motion_mode == 11) ? 2 : 0;
    } else {
      candidate = (isAttacked_ || trajectory.motion_mode == 11) ? 3 : 2;
    }
    motionModeHistory_.push_back(candidate);
    if (motionModeHistory_.size() > 40) motionModeHistory_.erase(motionModeHistory_.begin());
    const auto count = static_cast<int>(std::count(motionModeHistory_.begin(), motionModeHistory_.end(), candidate));
    if (count > 38) lastMotionMode_ = candidate;
    return static_cast<uint8_t>(lastMotionMode_);
  }

  void publishArrival(bool arrived) {
    std_msgs::msg::Bool message;
    message.data = arrived;
    arrivedPub_->publish(message);
  }

  void publishArrivedStop() {
    publishArrival(true);
    sentry_msgs::msg::SlaverSpeed command;
    command.line_speed = 0.0f;
    command.angle_target = 0.0f;
    command.angle_current = 0.0f;
    command.xtl_flag = selectMotionMode(true, false, trajectory_);
    command.in_bridge = 0;
    speedPub_->publish(command);
    std_msgs::msg::Bool status; status.data = true; solverStatusPub_->publish(status);
    std_msgs::msg::Bool replan; replan.data = false; replanPub_->publish(replan);
  }

  void stop(const char* reason) {
    solver_->reset();
    publishArrival(false);
    sentry_msgs::msg::SlaverSpeed command;
    command.line_speed = 0.0f; command.angle_target = 0.0f; command.angle_current = 0.0f;
    speedPub_->publish(command);
    std_msgs::msg::Bool status; status.data = false; solverStatusPub_->publish(status);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "trajectory tracking stopped: %s", reason);
  }

  void trajectoryCallback(const trajectory_generation::msg::TrajectoryPoly::SharedPtr message) {
    const std::size_t pieces = message->duration.size();
    if (pieces == 0 || message->coef_x.size() != 4 * pieces || message->coef_y.size() != 4 * pieces) {
      std::lock_guard<std::mutex> lock(mutex_); trajectory_.valid = false; stop("invalid coefficient array length"); return;
    }
    trajectory_tracking::Polynomial parsed;
    parsed.duration.assign(message->duration.begin(), message->duration.end());
    parsed.x.assign(message->coef_x.begin(), message->coef_x.end());
    parsed.y.assign(message->coef_y.begin(), message->coef_y.end());
    parsed.motion_mode = message->motion_mode;
    parsed.valid = trajectory_tracking::validatePolynomial(parsed);
    if (!parsed.valid) { RCLCPP_ERROR(get_logger(), "invalid trajectory pieces=%zu duration0=%f x0=%f y0=%f", pieces, parsed.duration[0], parsed.x[0], parsed.y[0]); std::lock_guard<std::mutex> lock(mutex_); trajectory_.valid = false; stop("non-finite trajectory"); return; }
    parsed.valid = true;
    if (!planner_.setTrajectory(parsed)) { stop("unable to initialize LocalPlanner"); return; }
    { std::lock_guard<std::mutex> lock(mutex_); trajectory_ = std::move(parsed); trajectoryStart_ = now(); lastReplanRequest_ = trajectoryStart_; replanHistory_.clear(); }
    solver_->reset();
    publishArrival(false);
  }

  bool sample(const trajectory_tracking::Polynomial& trajectory, double elapsed, Eigen::Vector4d& state) const {
    return trajectory_tracking::samplePolynomial(trajectory, elapsed, state);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
    trajectory_tracking::Polynomial current;
    rclcpp::Time start;
    { std::lock_guard<std::mutex> lock(mutex_); current = trajectory_; start = trajectoryStart_; }
    if (!current.valid) { stop("no valid trajectory"); return; }

    const double yaw = yawOf(odometry->pose.pose.orientation);
    std_msgs::msg::Float64 yawMessage;
    yawMessage.data = yaw;
    robotYawPub_->publish(yawMessage);
    if (gridMapReady_) {
      gridMap_->odom_position << odometry->pose.pose.position.x, odometry->pose.pose.position.y,
        odometry->pose.pose.position.z;
    }
    const auto& body = odometry->twist.twist.linear;
    const double worldVx = body.x * std::cos(yaw) - body.y * std::sin(yaw);
    const double worldVy = body.x * std::sin(yaw) + body.y * std::cos(yaw);
    const Eigen::Vector2d position2d(odometry->pose.pose.position.x, odometry->pose.pose.position.y);
    const Eigen::Vector2d target = planner_.target();
    const double speed = std::hypot(worldVx, worldVy);
    velocityHeading_ = trajectory_tracking::updateVelocityHeading(
      worldVx, worldVy, position2d, target, target.norm() > 0.1, velocityHeading_, velocityEpsilon_);
    ocs2::vector_t observation(4);
    observation << position2d.x(), position2d.y(), speed, velocityHeading_.heading;
    if (!observation.allFinite()) { stop("non-finite observation"); return; }

    const bool arrived = trajectory_tracking::hasArrived(position2d, target, current.motion_mode, arrivalDistance_);
    if (arrived) {
      publishArrivedStop();
      return;
    }

    trajectory_tracking::LocalPlanner::Reference reference;
    const Eigen::Vector3d position(observation(0), observation(1), 0.0);
    if (!planner_.makeFightReference(position, (now() - start).seconds(), yaw, reference)) {
      stop("invalid LocalPlanner reference"); return;
    }
    const bool haveReferencePhi = !reference.states.empty() && reference.states.front().allFinite();
    observation(3) = trajectory_tracking::observationHeading(
      velocityHeading_, speed, haveReferencePhi, haveReferencePhi ? reference.states.front()(3) : 0.0,
      referenceOverrideSpeed_);
    if (speed < referenceOverrideSpeed_ && haveReferencePhi) velocityHeading_.heading = observation(3);
    publishArrival(false);
    ocs2::vector_array_t states, inputs;
    states.reserve(reference.states.size());
    inputs.reserve(reference.states.size());
    for (const auto& state : reference.states) {
      states.emplace_back(state);
      inputs.emplace_back(Eigen::Vector2d::Zero());
    }
    updateCollisionConstraints(reference);
    robot_.getReferenceManagerPtr()->setTargetTrajectories(
      ocs2::TargetTrajectories(reference.times, states, inputs));

    try {
      if (!solver_->run(0.0, observation)) { stop("SqpMpc returned false"); return; }
      const auto solution = solver_->getSolverPtr()->primalSolution(0.0);
      if (solution.stateTrajectory_.size() < 2 || !solution.stateTrajectory_[1].allFinite()) { stop("invalid primal solution"); return; }
      const auto& predicted = solution.stateTrajectory_[1];
      double vx = predicted(2) * std::cos(predicted(3)), vy = predicted(2) * std::sin(predicted(3));
      const double magnitude = std::hypot(vx, vy);
      if (!isFinite(vx) || !isFinite(vy)) { stop("non-finite output"); return; }
      if (magnitude > outputMaxSpeed_) { const double scale = outputMaxSpeed_ / magnitude; vx *= scale; vy *= scale; }

      sentry_msgs::msg::SlaverSpeed command;
      // ROS1 uses line_speed as an unused compatibility field and transmits the
      // actual base_link command only through angle_target/angle_current.
      command.line_speed = 0.0f;
      command.angle_target = static_cast<float>(vx * std::cos(yaw) + vy * std::sin(yaw));
      command.angle_current = static_cast<float>(-vx * std::sin(yaw) + vy * std::cos(yaw));
      const bool inBridge = predictedInBridge(solution);
      command.xtl_flag = selectMotionMode(false, inBridge, current);
      command.in_bridge = inBridge ? 1 : 0;
      speedPub_->publish(command);
      std_msgs::msg::Bool status; status.data = true; solverStatusPub_->publish(status);
      publishVisualization(solution, reference);
      const bool collision = predictedDynamicObstacle(solution);
      updateReplanFlag(collision || reference.off_course, start, reference.redecision);
    } catch (const std::exception& error) { stop(error.what()); }
  }

  static geometry_msgs::msg::PoseStamped poseFromState(const Eigen::Vector4d& state, const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = stamp;
    pose.pose.position.x = state(0);
    pose.pose.position.y = state(1);
    pose.pose.orientation.z = std::sin(state(3) * 0.5);
    pose.pose.orientation.w = std::cos(state(3) * 0.5);
    return pose;
  }

  void publishVisualization(const ocs2::PrimalSolution& solution,
                            const trajectory_tracking::LocalPlanner::Reference& reference) {
    const auto stamp = now();
    nav_msgs::msg::Path predicted, referencePath;
    predicted.header.frame_id = referencePath.header.frame_id = "map";
    predicted.header.stamp = referencePath.header.stamp = stamp;
    for (const auto& state : solution.stateTrajectory_) {
      if (state.size() >= 4 && state.allFinite()) predicted.poses.push_back(poseFromState(state, stamp));
    }
    for (const auto& state : reference.states) referencePath.poses.push_back(poseFromState(state, stamp));
    predictedPathPub_->publish(predicted);
    referencePathPub_->publish(referencePath);

    const auto marker = [stamp](const char* ns, float r, float g, float b) {
      visualization_msgs::msg::Marker message;
      message.header.frame_id = "map";
      message.header.stamp = stamp;
      message.ns = ns;
      message.id = 0;
      message.type = visualization_msgs::msg::Marker::CUBE_LIST;
      message.action = visualization_msgs::msg::Marker::ADD;
      message.pose.orientation.w = 1.0;
      message.scale.x = message.scale.y = message.scale.z = 0.08;
      message.color.a = 1.0;
      message.color.r = r; message.color.g = g; message.color.b = b;
      return message;
    };
    auto candidate = marker("candidate_path", 1.0f, 0.2f, 0.0f);
    for (const auto& pose : predicted.poses) candidate.points.push_back(pose.pose.position);
    candidateMarkerPub_->publish(candidate);
    auto referenceMarker = marker("reference_path", 0.0f, 1.0f, 0.0f);
    for (const auto& pose : referencePath.poses) referenceMarker.points.push_back(pose.pose.position);
    referenceMarkerPub_->publish(referenceMarker);
    auto obstacles = marker("obstacle_centers", 1.0f, 0.0f, 0.0f);
    if (robot_.obsConstraintPtr_) {
      for (const auto& step : robot_.obsConstraintPtr_->obs_points_t_) {
        for (const auto& obstacle : step) {
          geometry_msgs::msg::Point point;
          point.x = obstacle.second.x(); point.y = obstacle.second.y(); point.z = obstacle.second.z();
          obstacles.points.push_back(point);
        }
      }
    }
    obstacleMarkerPub_->publish(obstacles);
  }

  void updateCollisionConstraints(const trajectory_tracking::LocalPlanner::Reference& reference) {
    if (!gridMapReady_ || !robot_.obsConstraintPtr_) return;
    std::vector<std::vector<std::pair<int, Eigen::Vector3d>>> points(reference.states.size());
    constexpr int kSearchHalfCells = 20;  // ROS1 LocalPlanner: +/- 1 m at 0.05 m.
    constexpr int kSectors = 8;
    for (std::size_t step = 0; step < reference.states.size(); ++step) {
      const auto center = gridMap_->coord2gridIndex(Eigen::Vector3d(reference.states[step](0), reference.states[step](1), 0.0));
      struct Candidate { double distance2; int type; Eigen::Vector3d point; bool valid; };
      std::array<Candidate, kSectors> sectors;
      for (auto& sector : sectors) sector = {0.0, 0, Eigen::Vector3d::Zero(), false};
      for (int dx = -kSearchHalfCells; dx <= kSearchHalfCells; ++dx) {
        for (int dy = -kSearchHalfCells; dy <= kSearchHalfCells; ++dy) {
          const int x = center.x() + dx, y = center.y() + dy;
          // RM_GridMap treats out-of-map as occupied; do not index its backing
          // arrays for those virtual boundary cells.
          if (x < 0 || x >= gridMap_->GLX_SIZE || y < 0 || y >= gridMap_->GLY_SIZE) continue;
          if (!gridMap_->isOccupied(x, y, center.z(), false)) continue;
          const Eigen::Vector3d point = gridMap_->gridIndex2coord(Eigen::Vector3i(x, y, center.z()));
          const double px = point.x() - reference.states[step](0);
          const double py = point.y() - reference.states[step](1);
          const double distance2 = px * px + py * py;
          if (distance2 > 1.0) continue;
          const double angle = std::atan2(py, px) + M_PI;
          const int sector = std::min(static_cast<int>(angle / (2.0 * M_PI) * kSectors), kSectors - 1);
          const int type = gridMap_->isLocalOccupied(x, y, center.z()) && gridMap_->data[x * gridMap_->GLY_SIZE + y] != 1 ? 1 : 0;
          if (!sectors[sector].valid || distance2 < sectors[sector].distance2)
            sectors[sector] = {distance2, type, point, true};
        }
      }
      for (const auto& sector : sectors) if (sector.valid) points[step].emplace_back(sector.type, sector.point);
    }
    robot_.obsConstraintPtr_->timeTrajectory_ = reference.times;
    robot_.obsConstraintPtr_->obs_points_t_ = std::move(points);
  }

  bool predictedDynamicObstacle(const ocs2::PrimalSolution& solution) const {
    if (!gridMapReady_) return false;
    const std::size_t limit = std::max<std::size_t>(2, solution.stateTrajectory_.size() / 4);
    for (std::size_t i = 1; i < std::min(limit, solution.stateTrajectory_.size()); ++i) {
      const auto& state = solution.stateTrajectory_[i];
      const auto index = gridMap_->coord2gridIndex(Eigen::Vector3d(state(0), state(1), 0.0));
      if (gridMap_->isLocalOccupied(index)) return true;
    }
    return false;
  }

  void updateReplanFlag(bool collision, const rclcpp::Time& trajectoryStart, bool redecision) {
    replanHistory_.push_back(collision);
    if (replanHistory_.size() > 41) replanHistory_.erase(replanHistory_.begin());
    const int collisions = static_cast<int>(std::count(replanHistory_.begin(), replanHistory_.end(), true));
    const double age = (now() - trajectoryStart).seconds();
    // Preserve the ROS1 policy: 41 collision observations after a 3s settle
    // window, plus the 5s periodic refresh for cleared dynamic obstacles.
    const bool periodic = (now() - lastReplanRequest_).seconds() > 5.0;
    const bool request = replanNow_ || (collisions > 40 && age > 3.0) || periodic;
    if (request) { replanNow_ = false; lastReplanRequest_ = now(); }
    std_msgs::msg::Bool replan;
    replan.data = request;
    replanPub_->publish(replan);
    std_msgs::msg::Bool redecisionMessage;
    redecisionMessage.data = redecision;
    redecisionPub_->publish(redecisionMessage);
    if (request) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "RM_GridMap predicted-path replan requested: collision_samples=%d trajectory_age=%.2f", collisions, age);
    }
  }

  void alignedPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (!gridMapReady_) return;
    try {
      sensor_msgs::msg::PointCloud2 mapCloud;
      if (cloud->header.frame_id == "map") {
        mapCloud = *cloud;
      } else {
        // A non-map cloud is admissible only after an explicit TF conversion.
        // This keeps body-frame scans out of the world-frame RM_GridMap.
        const auto transform = tfBuffer_->lookupTransform(
          "map", cloud->header.frame_id, cloud->header.stamp, rclcpp::Duration::from_seconds(0.1));
        tf2::doTransform(*cloud, mapCloud, transform);
      }
      pcl::PointCloud<pcl::PointXYZ> points;
      pcl::fromROSMsg(mapCloud, points);
      gridMap_->localPointCloudToObstacle(points, true, gridMap_->odom_position);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "dropping /aligned_points without map TF: %s", error.what());
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "dropping malformed /aligned_points: %s", error.what());
    }
  }

  std::mutex mutex_;
  trajectory_tracking::Polynomial trajectory_;
  rclcpp::Time trajectoryStart_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastReplanRequest_{0, 0, RCL_ROS_TIME};
  std::string taskFile_;
  double outputMaxSpeed_{2.0}, referenceDt_{0.1}, velocityEpsilon_{0.03}, referenceOverrideSpeed_{0.3};
  double arrivalDistance_{0.3}, collisionTimeStep_{0.1}, collisionPenaltyMu_{20.0}, collisionPenaltyDelta_{0.5};
  int collisionHorizonSteps_{20};
  trajectory_tracking::HeadingState velocityHeading_;
  trajectory_tracking::LocalPlanner planner_;
  std::unique_ptr<TrackingGridMap> gridMap_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  bool gridMapReady_{false};
  std::vector<bool> replanHistory_;
  bool teamIsRed_{true}, isXtl_{false}, isAttacked_{false}, replanNow_{false};
  int mateOutpostHp_{1500}, sentryHp_{0}, hpStableCount_{0};
  double wheelSpeed_{0.0}, wheelYaw_{0.0};
  int lastMotionMode_{0};
  std::vector<int> motionModeHistory_;
  SentryRobotInterface robot_;
  std::unique_ptr<ocs2::SqpMpc> solver_;
  rclcpp::Subscription<trajectory_generation::msg::TrajectoryPoly>::SharedPtr trajectorySub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr alignedPointsSub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wheelStateSub_;
  rclcpp::Subscription<sentry_msgs::msg::RobotStatus>::SharedPtr robotStatusSub_;
  rclcpp::Subscription<sentry_msgs::msg::RobotsHP>::SharedPtr robotHpSub_;
  rclcpp::Publisher<sentry_msgs::msg::SlaverSpeed>::SharedPtr speedPub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr solverStatusPub_, replanPub_, redecisionPub_, arrivedPub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr robotYawPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predictedPathPub_, referencePathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr candidateMarkerPub_, referenceMarkerPub_, obstacleMarkerPub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
