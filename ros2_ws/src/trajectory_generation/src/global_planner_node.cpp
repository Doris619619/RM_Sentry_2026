#include "trajectory_generation/planner_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_generation/msg/trajectory_poly.hpp"

namespace trajectory_generation {

class GlobalPlannerNode final : public rclcpp::Node {
 public:
  GlobalPlannerNode() : Node("trajectory_generation") {
    declare_parameters();
    configure_core();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          odom_ = *message;
          have_odom_ = true;
          if (!last_path_.empty() && distance_to_path(current_position(), last_path_) > deviation_replan_distance_) {
            request_replan(false, "odometry deviation");
          }
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, sensor_qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          handle_point_cloud(*message);
        });
    direct_goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        direct_goal_topic_, rclcpp::QoS(10).reliable(),
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
          set_goal(message->point.x, message->point.y, message->point.z, "clicked point");
        });
    waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
        waypoints_topic_, rclcpp::QoS(10).reliable(), [this](nav_msgs::msg::Path::ConstSharedPtr message) {
          if (message->poses.empty()) {
            RCLCPP_WARN(get_logger(), "received an empty waypoint path");
            return;
          }
          const auto& point = message->poses.back().pose.position;
          set_goal(point.x, point.y, point.z, "waypoint path");
        });
    replan_sub_ = create_subscription<std_msgs::msg::Bool>(
        replan_topic_, rclcpp::QoS(10).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          if (message->data) request_replan(false, "replan flag");
        });
    trajectory_pub_ = create_publisher<msg::TrajectoryPoly>(trajectory_topic_, rclcpp::QoS(10).reliable());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10).reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(30), [this] { execute(); });
    RCLCPP_INFO(get_logger(), "global planner ready: odom=%s cloud=%s direct_goal=%s waypoint=%s trajectory=%s",
                odom_topic_.c_str(), point_cloud_topic_.c_str(), direct_goal_topic_.c_str(),
                waypoints_topic_.c_str(), trajectory_topic_.c_str());
  }

 private:
  void declare_parameters() {
    odom_topic_ = declare_parameter<std::string>("topics.odom", "/localization/odometry");
    point_cloud_topic_ = declare_parameter<std::string>("topics.point_cloud", "/filted_topic_3d");
    direct_goal_topic_ = declare_parameter<std::string>("topics.direct_goal", "/clicked_point");
    waypoints_topic_ = declare_parameter<std::string>("topics.waypoints", "/waypoint_generator/waypoints");
    replan_topic_ = declare_parameter<std::string>("topics.replan", "/replan_flag");
    trajectory_topic_ = declare_parameter<std::string>("topics.global_trajectory", "/global_trajectory");
    path_topic_ = declare_parameter<std::string>("topics.global_path", "/global_path");
    marker_topic_ = declare_parameter<std::string>("topics.path_marker", "/astar_path_vis");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
    map_directory_ = declare_parameter<std::string>("maps.directory", "map");
    occupancy_file_ = declare_parameter<std::string>("maps.occupancy", "occfinal.png");
    bev_file_ = declare_parameter<std::string>("maps.bev", "bevfinal.png");
    topology_file_ = declare_parameter<std::string>("maps.topology", "occtopo.png");
    config_.resolution = declare_parameter<double>("planner.map_resolution", 0.05);
    config_.lower_x = declare_parameter<double>("planner.map_lower_x", -13.394);
    config_.lower_y = declare_parameter<double>("planner.map_lower_y", -12.079);
    config_.robot_radius = declare_parameter<double>("planner.robot_radius", 0.35);
    config_.desired_speed = declare_parameter<double>("planner.desired_speed", 2.0);
    config_.expected_map_width = declare_parameter<int>("maps.expected_width", 400);
    config_.expected_map_height = declare_parameter<int>("maps.expected_height", 400);
    config_.random_seed = declare_parameter<int>("planner.random_seed", -1);
    motion_mode_ = declare_parameter<int>("planner.motion_mode", 1);
    dynamic_obstacles_ = declare_parameter<bool>("planner.enable_dynamic_obstacles", true);
    dynamic_min_z_ = declare_parameter<double>("planner.dynamic_min_z", -0.05);
    dynamic_max_z_ = declare_parameter<double>("planner.dynamic_max_z", 1.2);
    replan_cooldown_ = declare_parameter<double>("planner.replan_cooldown_seconds", 1.0);
    deviation_replan_distance_ = declare_parameter<double>("planner.deviation_replan_distance", 1.0);
  }

  std::string resolve_map_path(const std::string& configured) const {
    const std::filesystem::path path(configured);
    if (path.is_absolute()) return path.string();
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory("trajectory_generation")) /
            map_directory_ / path).string();
  }

  void configure_core() {
    core_.configure(config_);
    core_.load_maps(resolve_map_path(occupancy_file_), resolve_map_path(bev_file_), resolve_map_path(topology_file_));
  }

  Eigen::Vector3d current_position() const {
    return {odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z};
  }

  void set_goal(double x, double y, double z, const char* source) {
    target_ = {x, y, z};
    have_target_ = true;
    plan_requested_ = true;
    RCLCPP_INFO(get_logger(), "received %s goal=(%.2f, %.2f, %.2f)", source, x, y, z);
  }

  void request_replan(bool force, const char* reason) {
    if (!have_target_ || !have_odom_) return;
    const auto elapsed = (now() - last_plan_time_).seconds();
    if (!force && last_plan_time_.nanoseconds() != 0 && elapsed < replan_cooldown_) {
      RCLCPP_DEBUG(get_logger(), "replan skipped during %.2fs cooldown: %s", replan_cooldown_ - elapsed, reason);
      return;
    }
    plan_requested_ = true;
    RCLCPP_INFO(get_logger(), "replan requested: %s", reason);
  }

  void handle_point_cloud(const sensor_msgs::msg::PointCloud2& message) {
    if (!dynamic_obstacles_) return;
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(map_frame_, message.header.frame_id, message.header.stamp,
                                               tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "dropping dynamic cloud without TF %s -> %s: %s", message.header.frame_id.c_str(),
                           map_frame_.c_str(), error.what());
      return;
    }
    tf2::Transform tf;
    tf2::fromMsg(transform.transform, tf);
    std::vector<Eigen::Vector3d> points;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) || *z < dynamic_min_z_ || *z > dynamic_max_z_) continue;
        const auto transformed = tf * tf2::Vector3(*x, *y, *z);
        points.emplace_back(transformed.x(), transformed.y(), transformed.z());
      }
    } catch (const std::runtime_error& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "dropping malformed dynamic cloud: %s", error.what());
      return;
    }
    core_.set_dynamic_obstacles(points);
    if (core_.path_collides(last_path_)) request_replan(false, "dynamic obstacle intersects path");
  }

  static double distance_to_path(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& path) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& candidate : path) nearest = std::min(nearest, (candidate - point).head<2>().norm());
    return nearest;
  }

  void execute() {
    if (!plan_requested_ || !have_odom_ || !have_target_) return;
    const auto result = core_.plan(current_position(), target_);
    plan_requested_ = false;
    if (result.smoothed_path.size() < 2 || result.trajectory.duration.empty()) {
      RCLCPP_ERROR(get_logger(), "global planning failed for start=(%.2f, %.2f) goal=(%.2f, %.2f)",
                   current_position().x(), current_position().y(), target_.x(), target_.y());
      return;
    }
    last_path_ = result.smoothed_path;
    last_plan_time_ = now();
    publish(result);
  }

  void publish(const PlanResult& result) {
    const auto stamp = now();
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "trajectory_generation";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.color.r = 0.1F;
    marker.color.g = 0.8F;
    marker.color.b = 0.2F;
    marker.color.a = 1.0F;
    for (const auto& point : result.smoothed_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.x();
      marker_point.y = point.y();
      marker_point.z = point.z();
      marker.points.push_back(marker_point);
    }
    msg::TrajectoryPoly trajectory;
    trajectory.start_time.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000LL);
    trajectory.start_time.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
    trajectory.motion_mode = static_cast<std::uint8_t>(motion_mode_);
    trajectory.coef_x = result.trajectory.coef_x;
    trajectory.coef_y = result.trajectory.coef_y;
    trajectory.duration = result.trajectory.duration;
    path_pub_->publish(path);
    marker_pub_->publish(marker);
    trajectory_pub_->publish(trajectory);
    RCLCPP_INFO(get_logger(), "published trajectory: raw=%zu topo=%zu smooth=%zu pieces=%zu",
                result.raw_path.size(), result.topological_path.size(), result.smoothed_path.size(),
                result.trajectory.duration.size());
  }

  PlannerConfig config_;
  PlannerCore core_;
  std::string odom_topic_, point_cloud_topic_, direct_goal_topic_, waypoints_topic_, replan_topic_;
  std::string trajectory_topic_, path_topic_, marker_topic_, map_frame_, base_frame_, map_directory_;
  std::string occupancy_file_, bev_file_, topology_file_;
  int motion_mode_{1};
  bool dynamic_obstacles_{true};
  double dynamic_min_z_{-0.05}, dynamic_max_z_{1.2}, replan_cooldown_{1.0}, deviation_replan_distance_{1.0};
  bool have_odom_{false}, have_target_{false}, plan_requested_{false};
  nav_msgs::msg::Odometry odom_;
  Eigen::Vector3d target_{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> last_path_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr direct_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_sub_;
  rclcpp::Publisher<msg::TrajectoryPoly>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace trajectory_generation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<trajectory_generation::GlobalPlannerNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_generation"), "planner startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
