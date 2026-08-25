#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace waypoint_generator {

class WaypointGeneratorNode : public rclcpp::Node {
 public:
  WaypointGeneratorNode()
      : Node("waypoint_generator", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    mode_ = declare_parameter<std::string>("waypoint_type", "manual-lonely-waypoint");
    goal_topic_ = declare_parameter<std::string>("topics.goal", "/goal");
    odom_topic_ = declare_parameter<std::string>("topics.odom", "/localization/odometry");
    trigger_topic_ = declare_parameter<std::string>("topics.trigger", "/traj_start_trigger");
    output_topic_ = declare_parameter<std::string>("topics.waypoints", "/waypoint_generator/waypoints");
    visual_topic_ = declare_parameter<std::string>("topics.visualization", "/waypoint_generator/waypoints_vis");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    radius_ = declare_parameter<double>("circle.radius", 1.0);
    count_ = declare_parameter<int>("circle.count", 24);
    segment_count_ = declare_parameter<int>("segment_count", 0);
    if (radius_ <= 0.0 || count_ < 2 || segment_count_ < 0) {
      throw std::invalid_argument("invalid waypoint generator radius, count, or segment_count");
    }
    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_topic_, 10);
    poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(visual_topic_, 10);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr goal) { on_goal(*goal); });
    trigger_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        trigger_topic_, 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr trigger) { on_trigger(*trigger); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
          current_pose_ = odom->pose.pose;
          odom_stamp_ = rclcpp::Time(odom->header.stamp);
          have_odom_ = true;
          publish_due_segments();
        });
    RCLCPP_INFO(get_logger(), "waypoint generator mode=%s output=%s", mode_.c_str(), output_topic_.c_str());
  }

 private:
  geometry_msgs::msg::Pose pose(double x, double y, double z, double yaw) const {
    geometry_msgs::msg::Pose result;
    result.position.x = x;
    result.position.y = y;
    result.position.z = z;
    result.orientation.z = std::sin(yaw * 0.5);
    result.orientation.w = std::cos(yaw * 0.5);
    return result;
  }

  void load_series(const rclcpp::Time& time_base) {
    segments_.clear();
    const double base_yaw = have_odom_ ? tf2::getYaw(current_pose_.orientation) : 0.0;
    for (int id = 0; id < segment_count_; ++id) {
      const std::string prefix = "seg" + std::to_string(id) + ".";
      double yaw = 0.0, offset = 0.0;
      std::vector<double> x, y, z;
      if (!get_parameter(prefix + "yaw", yaw) || !get_parameter(prefix + "time_from_start", offset) ||
          !get_parameter(prefix + "x", x) || !get_parameter(prefix + "y", y) || !get_parameter(prefix + "z", z) ||
          yaw <= -3.1499999 || yaw >= 3.1499999 || offset < 0.0 || x.empty() || x.size() != y.size() || x.size() != z.size()) {
        RCLCPP_ERROR(get_logger(), "invalid ROS1-compatible segment %d", id);
        segments_.clear();
        return;
      }
      nav_msgs::msg::Path segment;
      segment.header.frame_id = map_frame_;
      segment.header.stamp = time_base + rclcpp::Duration::from_seconds(offset);
      for (std::size_t index = 0; index < x.size(); ++index) {
        geometry_msgs::msg::PoseStamped point;
        point.header = segment.header;
        point.pose.orientation = pose(0.0, 0.0, 0.0, base_yaw + yaw).orientation;
        point.pose.position.x = std::cos(-base_yaw - yaw) * x[index] + std::sin(-base_yaw - yaw) * y[index] + current_pose_.position.x;
        point.pose.position.y = -std::sin(-base_yaw - yaw) * x[index] + std::cos(-base_yaw - yaw) * y[index] + current_pose_.position.y;
        point.pose.position.z = z[index] + current_pose_.position.z;
        segment.poses.push_back(point);
      }
      segments_.push_back(std::move(segment));
    }
  }

  void publish_due_segments() {
    while (!segments_.empty() && odom_stamp_ >= rclcpp::Time(segments_.front().header.stamp)) {
      publish(segments_.front());
      segments_.pop_front();
    }
  }

  nav_msgs::msg::Path legacy_pattern(const std::vector<std::array<double, 3>>& points) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (const auto& value : points) {
      geometry_msgs::msg::PoseStamped point;
      point.header = path.header;
      // ROS1 sample_waypoints.h uses a zero-yaw quaternion for every sample.
      point.pose = pose(value[0], value[1], value[2], 0.0);
      path.poses.push_back(point);
    }
    return path;
  }

  nav_msgs::msg::Path legacy_point() const {
    return legacy_pattern({{{14.0, 0.0, 1.0}, {28.0, 0.0, 1.0}, {35.0, 1.75, 1.0},
                            {37.1, 3.5, 1.0}, {35.0, 5.25, 1.0}, {28.0, 7.0, 1.0},
                            {14.0, 7.0, 1.0}, {0.0, 7.0, 1.0}}});
  }

  nav_msgs::msg::Path legacy_circle() const {
    return legacy_pattern({{{12.5, -6.0, 1.0}, {25.0, -12.0, 1.0}, {25.0, 0.0, 1.0},
                            {12.5, -6.0, 1.0}, {0.0, -12.0, 1.0}, {0.0, 0.0, 1.0},
                            {12.5, -6.0, 1.0}, {25.0, -12.0, 1.0}, {25.0, 0.0, 1.0},
                            {12.5, -6.0, 1.0}, {0.0, -12.0, 1.0}, {0.0, 0.0, 1.0}}});
  }

  nav_msgs::msg::Path legacy_eight() const {
    return legacy_pattern({{{10.0, -10.0, 1.0}, {20.0, 0.0, 2.0}, {30.0, 10.0, 1.0},
                            {40.0, 0.0, 2.0}, {30.0, -10.0, 1.0}, {20.0, 0.0, 2.0},
                            {10.0, 10.0, 1.0}, {0.0, 0.0, 2.0}, {10.0, -10.0, 3.0},
                            {20.0, 0.0, 2.0}, {30.0, 10.0, 3.0}, {40.0, 0.0, 2.0},
                            {30.0, -10.0, 3.0}, {20.0, 0.0, 2.0}, {10.0, 10.0, 3.0},
                            {0.0, 0.0, 2.0}}});
  }

  void on_goal(const geometry_msgs::msg::PoseStamped& goal) {
    if (mode_ == "manual-lonely-waypoint") {
      if (goal.pose.position.z < 0.0) {
        RCLCPP_WARN(get_logger(), "invalid manual-lonely waypoint (z < 0)");
        return;
      }
      nav_msgs::msg::Path path;
      path.header.stamp = now();
      path.header.frame_id = goal.header.frame_id.empty() ? map_frame_ : goal.header.frame_id;
      auto point = goal;
      point.header = path.header;
      path.poses.push_back(point);
      publish(std::move(path));
      return;
    }
    if (mode_ == "manual" || mode_ == "manual-waypoints") {
      if (goal.pose.position.z < -1.0) { publish(manual_path_); return; }
      if (goal.pose.position.z < 0.0) {
        if (!manual_path_.poses.empty()) manual_path_.poses.pop_back();
      } else {
        auto point = goal;
        point.header.frame_id = map_frame_;
        point.header.stamp = now();
        manual_path_.header = point.header;
        manual_path_.poses.push_back(point);
      }
      publish_visualization_only(manual_path_);
      return;
    }
    if (mode_ == "series") load_series(now());
    else if (mode_ == "circle") publish(legacy_circle());
    else if (mode_ == "eight") publish(legacy_eight());
    else if (mode_ == "point") publish(legacy_point());
    else if (mode_ == "noyaw" && goal.pose.position.z > 0.0) {
      auto point = goal;
      point.pose = pose(point.pose.position.x, point.pose.position.y, point.pose.position.z,
                        have_odom_ ? tf2::getYaw(current_pose_.orientation) : 0.0);
      manual_path_.poses.push_back(point);
      publish_visualization_only(manual_path_);
    }
  }

  void on_trigger(const geometry_msgs::msg::PoseStamped& trigger) {
    (void)trigger;
    if (mode_ == "free" || mode_ == "point") publish(legacy_point());
    else if (mode_ == "circle") publish(legacy_circle());
    else if (mode_ == "eight") publish(legacy_eight());
    else if (mode_ == "series") {
      if (!have_odom_) {
        RCLCPP_ERROR(get_logger(), "No odom for series waypoint trigger");
      } else {
        load_series(odom_stamp_);
      }
    }
  }

  void publish(nav_msgs::msg::Path path) {
    if (path.poses.empty()) return;
    if (path.header.frame_id.empty()) path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (auto& point : path.poses) point.header = path.header;
    geometry_msgs::msg::PoseArray array;
    array.header = path.header;
    for (const auto& point : path.poses) array.poses.push_back(point.pose);
    path_pub_->publish(path);
    poses_pub_->publish(array);
  }

  void publish_visualization_only(nav_msgs::msg::Path path) {
    if (path.header.frame_id.empty()) path.header.frame_id = map_frame_;
    path.header.stamp = now();
    geometry_msgs::msg::PoseArray array;
    array.header = path.header;
    for (const auto& point : path.poses) array.poses.push_back(point.pose);
    poses_pub_->publish(array);
  }

  std::string mode_, goal_topic_, odom_topic_, trigger_topic_, output_topic_, visual_topic_, map_frame_;
  double radius_{1.0};
  int count_{24};
  int segment_count_{0};
  bool have_odom_{false};
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path manual_path_;
  std::deque<nav_msgs::msg::Path> segments_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, trigger_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
};

}  // namespace waypoint_generator

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<waypoint_generator::WaypointGeneratorNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_generator"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
