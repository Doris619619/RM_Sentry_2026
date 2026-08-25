#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace waypoint_generator {

class WaypointGeneratorNode : public rclcpp::Node {
 public:
  WaypointGeneratorNode() : Node("waypoint_generator") {
    mode_ = declare_parameter<std::string>("waypoint_type", "manual-lonely-waypoint");
    goal_topic_ = declare_parameter<std::string>("topics.goal", "/goal");
    odom_topic_ = declare_parameter<std::string>("topics.odom", "/localization/odometry");
    trigger_topic_ = declare_parameter<std::string>("topics.trigger", "/traj_start_trigger");
    output_topic_ = declare_parameter<std::string>("topics.waypoints", "/waypoint_generator/waypoints");
    visual_topic_ = declare_parameter<std::string>("topics.visualization", "/waypoint_generator/waypoints_vis");
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    radius_ = declare_parameter<double>("circle.radius", 1.0);
    count_ = declare_parameter<int>("circle.count", 24);
    sequence_ = declare_parameter<std::vector<double>>("sequence", {});
    if (radius_ <= 0.0 || count_ < 2 || sequence_.size() % 4 != 0) {
      throw std::invalid_argument("invalid waypoint generator radius, count, or sequence (x,y,z,yaw tuples required)");
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
          have_odom_ = true;
        });
    if (mode_ == "series" && !sequence_.empty()) publish(sequence_path());
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

  nav_msgs::msg::Path sequence_path() const {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (std::size_t i = 0; i < sequence_.size(); i += 4) {
      geometry_msgs::msg::PoseStamped point;
      point.header = path.header;
      point.pose = pose(sequence_[i], sequence_[i + 1], sequence_[i + 2], sequence_[i + 3]);
      path.poses.push_back(point);
    }
    return path;
  }

  nav_msgs::msg::Path patterned_path(const geometry_msgs::msg::Pose& center, bool figure_eight) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (int i = 0; i <= count_; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count_);
      const double x = center.position.x + radius_ * std::cos(angle);
      const double y = center.position.y + (figure_eight ? radius_ * std::sin(2.0 * angle) * 0.5 : radius_ * std::sin(angle));
      geometry_msgs::msg::PoseStamped point;
      point.header = path.header;
      point.pose = pose(x, y, center.position.z, angle + M_PI_2);
      path.poses.push_back(point);
    }
    return path;
  }

  void on_goal(const geometry_msgs::msg::PoseStamped& goal) {
    if (mode_ == "manual-lonely-waypoint") {
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
      publish(manual_path_);
      return;
    }
    if (mode_ == "series") publish(sequence_path());
    else if (mode_ == "circle") publish(patterned_path(goal.pose, false));
    else if (mode_ == "eight") publish(patterned_path(goal.pose, true));
    else if (mode_ == "point") {
      nav_msgs::msg::Path path;
      path.header = goal.header;
      path.header.frame_id = map_frame_;
      path.header.stamp = now();
      path.poses.push_back(goal);
      publish(std::move(path));
    }
  }

  void on_trigger(const geometry_msgs::msg::PoseStamped& trigger) {
    if (mode_ != "circle" && mode_ != "eight") return;
    const auto& center = have_odom_ ? current_pose_ : trigger.pose;
    publish(patterned_path(center, mode_ == "eight"));
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

  std::string mode_, goal_topic_, odom_topic_, trigger_topic_, output_topic_, visual_topic_, map_frame_;
  double radius_{1.0};
  int count_{24};
  std::vector<double> sequence_;
  bool have_odom_{false};
  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Path manual_path_;
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
