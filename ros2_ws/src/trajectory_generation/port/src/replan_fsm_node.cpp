// ROS2 adapter of the ROS1 ReplanFSM.  Its state transitions and calls into
// planner_manager are intentionally the same as replan_fsm.cpp; only ROS
// communication, parameter ownership, TF and publishing are replaced.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "trajectory_generation/plan_manager.h"
#include "trajectory_generation/visualization_utils.h"
#include "trajectory_generation/msg/trajectory_poly.hpp"

namespace trajectory_generation {

class ReplanFsmNode final : public rclcpp::Node {
 public:
  ReplanFsmNode() : Node("trajectory_generation"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    declare_parameters();
    initialise_manager();
    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          receive_odometry(*message);
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, sensor_qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          receive_dynamic_cloud(*message);
        });
    clicked_goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_goal_topic_, rclcpp::QoS(10), [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
          receive_goal(message->point.x, message->point.y, message->point.z, "clicked point");
        });
    pose_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_goal_topic_, rclcpp::QoS(10), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          receive_goal(message->pose.position.x, message->pose.position.y, message->pose.position.z, "manual goal");
        });
    waypoint_sub_ = create_subscription<nav_msgs::msg::Path>(
        waypoint_topic_, rclcpp::QoS(10), [this](nav_msgs::msg::Path::ConstSharedPtr message) {
          if (message->poses.empty()) {
            RCLCPP_WARN(get_logger(), "received empty waypoint path");
            return;
          }
          const auto& point = message->poses.back().pose.position;
          receive_goal(point.x, point.y, point.z, "waypoint path");
        });
    replan_sub_ = create_subscription<std_msgs::msg::Bool>(
        replan_topic_, rclcpp::QoS(10), [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          receive_replan_flag(message->data);
        });
    trajectory_pub_ = create_publisher<msg::TrajectoryPoly>(trajectory_topic_, rclcpp::QoS(10));
    target_pub_ = create_publisher<geometry_msgs::msg::Point>(target_topic_, rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::milliseconds(30), [this] { execute_fsm(); });
  }

 private:
  enum class State { kInit, kWaitTarget, kGenerateNewTrajectory, kReplanTrajectory, kExecuteTrajectory };

  void declare_parameters() {
    odom_topic_ = declare_parameter<std::string>("topics.odom", "/localization/odometry");
    cloud_topic_ = declare_parameter<std::string>("topics.point_cloud", "/filted_topic_3d");
    clicked_goal_topic_ = declare_parameter<std::string>("topics.clicked_goal", "/clicked_point");
    pose_goal_topic_ = declare_parameter<std::string>("topics.goal", "/goal");
    waypoint_topic_ = declare_parameter<std::string>("topics.waypoints", "/waypoint_generator/waypoints");
    replan_topic_ = declare_parameter<std::string>("topics.replan", "/replan_flag");
    trajectory_topic_ = declare_parameter<std::string>("topics.global_trajectory", "/global_trajectory");
    target_topic_ = declare_parameter<std::string>("topics.target_result", "/target_result");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    map_directory_ = declare_parameter<std::string>("maps.directory", "map");
    occupancy_file_ = declare_parameter<std::string>("maps.occupancy", "occfinal.png");
    bev_file_ = declare_parameter<std::string>("maps.bev", "bevfinal.png");
    topology_file_ = declare_parameter<std::string>("maps.topology", "occtopo.png");
    map_resolution_ = declare_parameter<double>("planner.map_resolution", 0.05);
    map_x_size_ = declare_parameter<double>("planner.map_x_size", 20.0);
    map_y_size_ = declare_parameter<double>("planner.map_y_size", 20.0);
    map_lower_x_ = declare_parameter<double>("planner.map_lower_x", -13.394);
    map_lower_y_ = declare_parameter<double>("planner.map_lower_y", -12.079);
    robot_radius_ = declare_parameter<double>("planner.robot_radius", 0.35);
    robot_radius_dash_ = declare_parameter<double>("planner.robot_radius_dash", 0.35);
    desired_speed_ = declare_parameter<double>("planner.reference_desire_speed", 2.0);
    motion_mode_ = declare_parameter<int>("planner.motion_mode", 1);
    replan_cooldown_seconds_ = declare_parameter<double>("planner.replan_cooldown_seconds", 1.0);
    enable_dynamic_cloud_ = declare_parameter<bool>("planner.enable_dynamic_cloud", true);
    random_seed_ = declare_parameter<int>("planner.test_random_seed", -1);
    expected_map_width_ = declare_parameter<int>("maps.expected_width", 400);
    expected_map_height_ = declare_parameter<int>("maps.expected_height", 400);
  }

  std::string map_path(const std::string& file) const {
    const std::filesystem::path configured(file);
    if (configured.is_absolute()) return configured.string();
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory("trajectory_generation")) /
            map_directory_ / configured).string();
  }

  void initialise_manager() {
    if (map_resolution_ <= 0.0 || map_x_size_ <= 0.0 || map_y_size_ <= 0.0 || robot_radius_ <= 0.0 ||
        expected_map_width_ <= 0 || expected_map_height_ <= 0) {
      throw std::invalid_argument("invalid global-planning map or robot parameters");
    }
    for (const auto& path : {map_path(occupancy_file_), map_path(bev_file_), map_path(topology_file_)}) {
      const auto image = cv::imread(path, cv::IMREAD_UNCHANGED);
      if (image.empty()) throw std::runtime_error("required planning map is missing: " + path);
      if (image.cols != expected_map_width_ || image.rows != expected_map_height_) {
        throw std::runtime_error("planning map dimensions do not match configured map dimensions: " + path);
      }
    }
    // These are the exact ROS1 parameter keys consumed by plan_manager and
    // RM_GridMap.  Values originate from declared ROS2 parameters above.
    bridge_.setParam("trajectory_generator/occ_file_path", map_path(occupancy_file_));
    bridge_.setParam("trajectory_generator/bev_file_path", map_path(bev_file_));
    bridge_.setParam("trajectory_generator/distance_map_file_path", map_path(topology_file_));
    bridge_.setParam("trajectory_generator/map_resolution", map_resolution_);
    bridge_.setParam("trajectory_generator/map_x_size", map_x_size_);
    bridge_.setParam("trajectory_generator/map_y_size", map_y_size_);
    bridge_.setParam("trajectory_generator/map_z_size", 2.0);
    bridge_.setParam("trajectory_generator/map_lower_point_x", map_lower_x_);
    bridge_.setParam("trajectory_generator/map_lower_point_y", map_lower_y_);
    bridge_.setParam("trajectory_generator/map_lower_point_z", 0.0);
    bridge_.setParam("trajectory_generator/robot_radius", robot_radius_);
    bridge_.setParam("trajectory_generator/robot_radius_dash", robot_radius_dash_);
    bridge_.setParam("trajectory_generator/reference_desire_speed", desired_speed_);
    bridge_.setParam("trajectory_generator/reference_desire_speedxtl", 2.4);
    bridge_.setParam("trajectory_generator/reference_v_max", 2.5);
    bridge_.setParam("trajectory_generator/reference_a_max", 4.0);
    bridge_.setParam("trajectory_generator/reference_w_max", 4.0);
    bridge_.setParam("trajectory_generator/reference_axtl_max", 2.0);
    bridge_.setParam("trajectory_generator/reference_wxtl_max", 2.0);
    bridge_.setParam("trajectory_generator/search_height_min", -0.05);
    bridge_.setParam("trajectory_generator/search_height_max", 1.2);
    bridge_.setParam("trajectory_generator/search_radius", 6.0);
    bridge_.setParam("trajectory_generator/obstacle_swell_flag", true);
    bridge_.setParam("trajectory_generator/isxtl", false);
    bridge_.setParam("trajectory_generator/xtl_flag", false);
    bridge_.setParam("trajectory_generator/height_bias", 0.015294117853045464);
    bridge_.setParam("trajectory_generator/height_interval", 1.5);
    bridge_.setParam("trajectory_generator/height_threshold", 0.08);
    bridge_.setParam("trajectory_generator/height_sencond_high_threshold", 0.2);
    manager_ = std::make_unique<planner_manager>();
    manager_->init(bridge_);
    if (random_seed_ >= 0) manager_->topo_prm->setRandomSeed(static_cast<unsigned int>(random_seed_));
    visualization_ = std::make_unique<Vislization>();
    visualization_->init(bridge_);
    bind_visualization_publishers();
  }

  void bind_visualization_publishers() {
    const auto bind = [this](ros::Publisher& legacy, const std::string& topic) {
      const auto publisher = create_publisher<visualization_msgs::msg::Marker>(topic, rclcpp::QoS(10));
      legacy.setMarkerPublisher([publisher](const visualization_msgs::msg::Marker& marker) { publisher->publish(marker); });
      visualization_publishers_.push_back(publisher);
    };
    bind(visualization_->astar_path_vis_pub, "astar_path_vis");
    bind(visualization_->final_path_vis_pub, "final_path_vis_pub");
    bind(visualization_->final_line_strip_pub, "final_line_strip_pub");
    bind(visualization_->optimized_path_vis_pub, "optimized_path_vis");
    bind(visualization_->cur_position_vis_pub, "cur_position_vis");
    bind(visualization_->target_position_vis_pub, "target");
    bind(visualization_->topo_position_guard_vis_pub, "topo_point_guard");
    bind(visualization_->topo_position_connection_vis_pub, "topo_point_connection");
    bind(visualization_->topo_line_vis_pub, "topo_line");
    bind(visualization_->reference_path_vis_pub, "reference_path");
    bind(visualization_->obs_vis_pub, "obs");
    bind(visualization_->topo_path_point_vis_pub, "topo_point");
    bind(visualization_->topo_path_vis_pub, "topo_point_path");
    bind(visualization_->minco_input_vis_pub, "minco_input_vis");
    bind(visualization_->minco_output_vis_pub, "minco_output_vis");
    bind(visualization_->reference_line_vis_pub, "reference_line_vis");
  }

  void receive_goal(double x, double y, double z, const char* source) {
    (void)z;  // ROS1 ground planner intentionally replaces input Z by map height.
    Eigen::Vector3d target(x, y, 0.0);
    auto goal_index = manager_->global_map->coord2gridIndex(target);
    if (manager_->global_map->isOccupied(goal_index, false)) {
      Eigen::Vector3i replacement;
      if (manager_->astar_path_finder->findNeighPoint(goal_index, replacement, 2)) goal_index = replacement;
    }
    final_goal_ = manager_->global_map->gridIndex2coord(goal_index);
    final_goal_.z() = manager_->global_map->getHeight(goal_index.x(), goal_index.y());
    have_target_ = true;
    state_ = State::kGenerateNewTrajectory;
    RCLCPP_INFO(get_logger(), "FSM accepted %s goal=(%.2f, %.2f, %.2f)", source, final_goal_.x(), final_goal_.y(), final_goal_.z());
  }

  void receive_odometry(const nav_msgs::msg::Odometry& state) {
    const auto& pose = state.pose.pose;
    robot_position_ = Eigen::Vector3d(pose.position.x, pose.position.y, 0.0);
    robot_speed_ = Eigen::Vector3d(state.twist.twist.linear.x, state.twist.twist.linear.y, 0.0);
    start_point_ = robot_position_;
    auto start_index = manager_->global_map->coord2gridIndex(start_point_);
    if (manager_->global_map->isOccupied(start_index, false)) {
      Eigen::Vector3i replacement;
      if (manager_->astar_path_finder->findNeighPoint(start_index, replacement, 2)) start_index = replacement;
    }
    const auto snapped = manager_->global_map->gridIndex2coord(start_index);
    start_point_.x() = snapped.x();
    start_point_.y() = snapped.y();
    have_odom_ = true;
  }

  void receive_dynamic_cloud(const sensor_msgs::msg::PointCloud2& cloud) {
    if (!enable_dynamic_cloud_ || !have_odom_) return;
    sensor_msgs::msg::PointCloud2 in_map;
    try {
      const auto transform = tf_buffer_.lookupTransform(map_frame_, cloud.header.frame_id, cloud.header.stamp,
                                                        tf2::durationFromSec(0.1));
      tf2::doTransform(cloud, in_map, transform);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "discarded dynamic cloud without %s <- %s TF: %s", map_frame_.c_str(),
                           cloud.header.frame_id.c_str(), error.what());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> points;
    pcl::fromROSMsg(in_map, points);
    manager_->global_map->m_local_cloud->clear();
    for (const auto& point : points.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      if (std::abs(point.x - robot_position_.x()) > 6.0 || std::abs(point.y - robot_position_.y()) > 6.0) continue;
      manager_->global_map->m_local_cloud->push_back(point);
    }
    manager_->global_map->localPointCloudToObstacle(*manager_->global_map->m_local_cloud, true, robot_position_);
    if (state_ == State::kExecuteTrajectory) receive_replan_flag(true);
  }

  void receive_replan_flag(bool requested) {
    if (!requested || manager_->optimized_path.size() < 2) {
      if (!requested) replan_active_ = false;
      return;
    }
    const double elapsed = (now() - last_replan_time_).seconds();
    if (last_replan_time_.nanoseconds() != 0 && elapsed < replan_cooldown_seconds_) {
      RCLCPP_DEBUG(get_logger(), "FSM replan cooldown %.2fs", replan_cooldown_seconds_ - elapsed);
      return;
    }
    if (!replan_active_) state_ = State::kReplanTrajectory;
  }

  void execute_fsm() {
    if (!have_odom_) return;  // original INIT waits; never publishes a synthetic trajectory
    visualization_->visAstarPath(manager_->astar_path);
    visualization_->visOptimizedPath(manager_->final_path);
    visualization_->visOptGlobalPath(manager_->ref_trajectory);
    visualization_->visFinalPath(manager_->optimized_path);
    visualization_->visCurPosition(robot_position_);
    visualization_->visTopoPointGuard(manager_->topo_prm->m_graph);
    visualization_->visTopoPointConnection(manager_->topo_prm->m_graph);
    visualization_->visMincoInput(manager_->minco_input_pts);
    visualization_->visMincoOutput(manager_->final_path);
    visualization_->visReferenceLine(manager_->ref_trajectory);
    if (state_ == State::kInit) state_ = State::kWaitTarget;
    if (state_ == State::kWaitTarget) {
      if (!have_target_) return;
      state_ = State::kGenerateNewTrajectory;
    }
    if (state_ == State::kGenerateNewTrajectory) {
      if (manager_->pathFinding(start_point_, final_goal_, robot_speed_)) {
        state_ = State::kExecuteTrajectory;
        publish_trajectory();
      } else {
        RCLCPP_ERROR(get_logger(), "FSM global planning failed");
      }
    }
    if (state_ == State::kReplanTrajectory) {
      if (manager_->replanFinding(robot_position_, final_goal_, robot_speed_)) {
        state_ = State::kExecuteTrajectory;
        last_replan_time_ = now();
        replan_active_ = true;
        publish_trajectory();
      } else {
        state_ = State::kGenerateNewTrajectory;
        RCLCPP_ERROR(get_logger(), "FSM replanning failed; retrying global planning");
      }
    }
  }

  void publish_trajectory() {
    msg::TrajectoryPoly output;
    output.start_time = now();
    output.motion_mode = static_cast<std::uint8_t>(motion_mode_);
    const auto& times = manager_->reference_path->m_trapezoidal_time;
    for (std::size_t index = 0; index < times.size(); ++index) {
      const double duration = times[index];
      if (!std::isfinite(duration) || duration <= 0.0) {
        RCLCPP_ERROR(get_logger(), "refusing invalid polynomial duration");
        return;
      }
      output.duration.push_back(static_cast<float>(duration));
      for (Eigen::Index coefficient = 0; coefficient < 4; ++coefficient) {
        output.coef_x.push_back(static_cast<float>(manager_->reference_path->m_polyMatrix_x(index, coefficient)));
        output.coef_y.push_back(static_cast<float>(manager_->reference_path->m_polyMatrix_y(index, coefficient)));
      }
    }
    if (output.duration.empty() || output.coef_x.size() != output.duration.size() * 4 ||
        output.coef_y.size() != output.duration.size() * 4) {
      RCLCPP_ERROR(get_logger(), "refusing malformed trajectory polynomial");
      return;
    }
    geometry_msgs::msg::Point target;
    target.x = final_goal_.x(); target.y = final_goal_.y(); target.z = final_goal_.z();
    target_pub_->publish(target);
    trajectory_pub_->publish(output);
    RCLCPP_INFO(get_logger(), "FSM published %zu trajectory pieces", output.duration.size());
  }

  ros::NodeHandle bridge_;
  std::unique_ptr<planner_manager> manager_;
  std::unique_ptr<Vislization> visualization_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  State state_{State::kInit};
  Eigen::Vector3d robot_position_{Eigen::Vector3d::Zero()}, robot_speed_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_point_{Eigen::Vector3d::Zero()}, final_goal_{Eigen::Vector3d::Zero()};
  bool have_odom_{false}, have_target_{false}, replan_active_{false}, enable_dynamic_cloud_{true};
  int motion_mode_{1};
  int random_seed_{-1}, expected_map_width_{400}, expected_map_height_{400};
  double map_resolution_{0.05}, map_x_size_{20.0}, map_y_size_{20.0}, map_lower_x_{-13.394}, map_lower_y_{-12.079};
  double robot_radius_{0.35}, robot_radius_dash_{0.35}, desired_speed_{2.0}, replan_cooldown_seconds_{1.0};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  std::string odom_topic_, cloud_topic_, clicked_goal_topic_, pose_goal_topic_, waypoint_topic_, replan_topic_;
  std::string trajectory_topic_, target_topic_, map_frame_, map_directory_, occupancy_file_, bev_file_, topology_file_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replan_sub_;
  rclcpp::Publisher<msg::TrajectoryPoly>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> visualization_publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace trajectory_generation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<trajectory_generation::ReplanFsmNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_generation"), "planner startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
