#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

#include <sentry_msgs/msg/slaver_speed.hpp>
#include <trajectory_generation/msg/trajectory_poly.hpp>
#include "ocs2_sentry/SentryRobotInterface.h"
#include "KMF.h"

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
    obstacleStopDistance_ = declare_parameter<double>("obstacle_stop_distance", 0.45);

    robot_.init(taskFile_);
    robot_.sqpSettings().hpipmSettings.hpipmMode = hpipm_mode::ROBUST;
    robot_.sqpSettings().hpipmSettings.reg_prim = 1e-8;
    solver_ = std::make_unique<ocs2::SqpMpc>(
      robot_.mpcSettings(), robot_.sqpSettings(), robot_.getOptimalControlProblem(), robot_.getInitializer());
    solver_->getSolverPtr()->setReferenceManager(robot_.getReferenceManagerPtr());

    trajectorySub_ = create_subscription<trajectory_generation::msg::TrajectoryPoly>(
      "/global_trajectory", rclcpp::QoS(10).reliable(),
      std::bind(&TrajectoryTrackingNode::trajectoryCallback, this, std::placeholders::_1));
    odometrySub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odometry", rclcpp::QoS(5).reliable(),
      std::bind(&TrajectoryTrackingNode::odometryCallback, this, std::placeholders::_1));
    alignedPointsSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/aligned_points", rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryTrackingNode::alignedPointsCallback, this, std::placeholders::_1));
    speedPub_ = create_publisher<sentry_msgs::msg::SlaverSpeed>("/sentry_des_speed", rclcpp::QoS(10).reliable());
    solverStatusPub_ = create_publisher<std_msgs::msg::Bool>("/solver_status", rclcpp::QoS(10).reliable());
    replanPub_ = create_publisher<std_msgs::msg::Bool>("/replan_flag", rclcpp::QoS(10).reliable());
  }

 private:
  struct Polynomial {
    std::vector<double> duration, x, y;
    uint8_t motionMode{0};
    rclcpp::Time start{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void stop(const char* reason) {
    solver_->reset();
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
    Polynomial parsed;
    parsed.duration.assign(message->duration.begin(), message->duration.end());
    parsed.x.assign(message->coef_x.begin(), message->coef_x.end());
    parsed.y.assign(message->coef_y.begin(), message->coef_y.end());
    parsed.motionMode = message->motion_mode;
    parsed.start = now();
    parsed.valid = true;
    for (std::size_t i = 0; i < pieces; ++i) {
      if (!(parsed.duration[i] > 0.0) || !std::isfinite(parsed.duration[i])) { parsed.valid = false; break; }
      for (std::size_t j = 0; j < 4; ++j) if (!std::isfinite(parsed.x[4 * i + j]) || !std::isfinite(parsed.y[4 * i + j])) parsed.valid = false;
    }
    if (!parsed.valid) { RCLCPP_ERROR(get_logger(), "invalid trajectory pieces=%zu duration0=%f x0=%f y0=%f", pieces, parsed.duration[0], parsed.x[0], parsed.y[0]); std::lock_guard<std::mutex> lock(mutex_); trajectory_.valid = false; stop("non-finite trajectory"); return; }
    parsed.valid = true;
    { std::lock_guard<std::mutex> lock(mutex_); trajectory_ = std::move(parsed); obstacleBlocked_ = false; }
    solver_->reset();
  }

  bool sample(const Polynomial& trajectory, double elapsed, Eigen::Vector4d& state) const {
    std::size_t piece = 0; double local = std::max(0.0, elapsed);
    while (piece + 1 < trajectory.duration.size() && local > trajectory.duration[piece]) { local -= trajectory.duration[piece++]; }
    local = std::min(local, trajectory.duration[piece]);
    const auto eval = [piece, local](const std::vector<double>& c) {
      const std::size_t i = 4 * piece;
      return c[i] + c[i + 1] * local + c[i + 2] * local * local + c[i + 3] * local * local * local;
    };
    const auto derivative = [piece, local](const std::vector<double>& c) {
      const std::size_t i = 4 * piece;
      return c[i + 1] + 2.0 * c[i + 2] * local + 3.0 * c[i + 3] * local * local;
    };
    const double vx = derivative(trajectory.x), vy = derivative(trajectory.y);
    state << eval(trajectory.x), eval(trajectory.y), std::hypot(vx, vy), std::atan2(vy, vx);
    return state.allFinite();
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
    Polynomial current;
    bool blocked;
    { std::lock_guard<std::mutex> lock(mutex_); current = trajectory_; blocked = obstacleBlocked_; }
    if (!current.valid || blocked) { stop(blocked ? "map-frame obstacle" : "no valid trajectory"); return; }

    const double yaw = yawOf(odometry->pose.pose.orientation);
    const auto& body = odometry->twist.twist.linear;
    const double worldVx = body.x * std::cos(yaw) - body.y * std::sin(yaw);
    const double worldVy = body.x * std::sin(yaw) + body.y * std::cos(yaw);
    ocs2::vector_t observation(4);
    const double speed = std::hypot(worldVx, worldVy);
    observation << odometry->pose.pose.position.x, odometry->pose.pose.position.y, speed,
      speed > 0.03 ? std::atan2(worldVy, worldVx) : yaw;
    if (!observation.allFinite()) { stop("non-finite observation"); return; }

    const double elapsed = (now() - current.start).seconds();
    ocs2::scalar_array_t times; ocs2::vector_array_t states, inputs;
    for (int i = 0; i < 20; ++i) {
      Eigen::Vector4d reference;
      if (!sample(current, elapsed + i * referenceDt_, reference)) { stop("non-finite sampled reference"); return; }
      times.push_back(i * referenceDt_);
      states.push_back(reference);
      inputs.push_back(Eigen::Vector2d::Zero());
    }
    robot_.getReferenceManagerPtr()->setTargetTrajectories(ocs2::TargetTrajectories(times, states, inputs));

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
      command.line_speed = static_cast<float>(std::hypot(vx, vy));
      command.angle_target = static_cast<float>(vx * std::cos(yaw) + vy * std::sin(yaw));
      command.angle_current = static_cast<float>(-vx * std::sin(yaw) + vy * std::cos(yaw));
      command.xtl_flag = 0; command.in_bridge = 0;
      speedPub_->publish(command);
      std_msgs::msg::Bool status; status.data = true; solverStatusPub_->publish(status);
      std_msgs::msg::Bool replan; replan.data = false; replanPub_->publish(replan);
    } catch (const std::exception& error) { stop(error.what()); }
  }

  void alignedPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (cloud->header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "dropping /aligned_points outside map frame: %s", cloud->header.frame_id.c_str());
      return;
    }
    Polynomial current;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current = trajectory_;
    }
    if (!current.valid) return;
    const double elapsed = (now() - current.start).seconds();
    std::vector<Eigen::Vector2d> reference;
    reference.reserve(20);
    for (int i = 0; i < 20; ++i) {
      Eigen::Vector4d sampled;
      if (!sample(current, elapsed + i * referenceDt_, sampled)) return;
      reference.emplace_back(sampled.x(), sampled.y());
    }
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      for (; x != x.end(); ++x, ++y) {
        if (!std::isfinite(*x) || !std::isfinite(*y)) continue;
        const Eigen::Vector2d point(*x, *y);
        for (const auto& target : reference) {
          if ((point - target).norm() <= obstacleStopDistance_) {
            {
              std::lock_guard<std::mutex> lock(mutex_);
              obstacleBlocked_ = true;
            }
            std_msgs::msg::Bool replan;
            replan.data = true;
            replanPub_->publish(replan);
            stop("map-frame obstacle reaches trajectory");
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "map-frame obstacle triggers a replan request");
            return;
          }
        }
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "dropping malformed /aligned_points: %s", error.what());
    }
  }

  std::mutex mutex_;
  Polynomial trajectory_;
  bool obstacleBlocked_{false};
  std::string taskFile_;
  double outputMaxSpeed_{2.0}, referenceDt_{0.1}, obstacleStopDistance_{0.45};
  SentryRobotInterface robot_;
  std::unique_ptr<ocs2::SqpMpc> solver_;
  rclcpp::Subscription<trajectory_generation::msg::TrajectoryPoly>::SharedPtr trajectorySub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr alignedPointsSub_;
  rclcpp::Publisher<sentry_msgs::msg::SlaverSpeed>::SharedPtr speedPub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr solverStatusPub_, replanPub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
