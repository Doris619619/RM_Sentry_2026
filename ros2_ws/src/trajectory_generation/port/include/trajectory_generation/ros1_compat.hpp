#pragma once

// Transitional ROS1 surface used only by the algorithm port.  It keeps the
// original search, map, smoothing, and reference-path code structurally
// intact while ROS2-facing nodes own subscriptions, parameters, TF, and
// publishing.  It is deliberately not used by the final ReplanFSM adapter.

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <any>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace geometry_msgs {
using Point = msg::Point;
using PointStamped = msg::PointStamped;
using Pose = msg::Pose;
using PoseArray = msg::PoseArray;
using PoseStamped = msg::PoseStamped;
using Vector3 = msg::Vector3;
using PointStampedConstPtr = std::shared_ptr<const PointStamped>;
using PoseStampedConstPtr = std::shared_ptr<const PoseStamped>;
using Vector3ConstPtr = std::shared_ptr<const Vector3>;
}  // namespace geometry_msgs
namespace nav_msgs {
using Odometry = msg::Odometry;
using Path = msg::Path;
using OdometryConstPtr = std::shared_ptr<const Odometry>;
}  // namespace nav_msgs
namespace sensor_msgs {
using PointCloud2 = msg::PointCloud2;
using PointCloud2ConstPtr = std::shared_ptr<const PointCloud2>;
}  // namespace sensor_msgs
namespace std_msgs {
using Bool = msg::Bool;
using Int64 = msg::Int64;
using BoolConstPtr = std::shared_ptr<const Bool>;
using Int64ConstPtr = std::shared_ptr<const Int64>;
}  // namespace std_msgs
namespace visualization_msgs {
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}  // namespace visualization_msgs
namespace gazebo_msgs {
struct ModelStates {
  using ConstPtr = std::shared_ptr<const ModelStates>;
  std::vector<std::string> name;
  std::vector<geometry_msgs::Pose> pose;
};
using ModelStatesConstPtr = ModelStates::ConstPtr;
}  // namespace gazebo_msgs

namespace ros {
class Duration {
 public:
  explicit Duration(double seconds = 0.0) : seconds_(seconds) {}
  [[nodiscard]] double toSec() const { return seconds_; }
 private:
  double seconds_;
};
class Time {
 public:
  Time() = default;
  explicit Time(double seconds) : seconds_(seconds) {}
  static Time now() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return Time(std::chrono::duration<double>(now).count());
  }
  [[nodiscard]] double toSec() const { return seconds_; }
  operator builtin_interfaces::msg::Time() const {
    builtin_interfaces::msg::Time value;
    value.sec = static_cast<std::int32_t>(seconds_);
    value.nanosec = static_cast<std::uint32_t>((seconds_ - value.sec) * 1000000000.0);
    return value;
  }
  friend Duration operator-(const Time& lhs, const Time& rhs) {
    return Duration(lhs.seconds_ - rhs.seconds_);
  }
 private:
  double seconds_{0.0};
};
class Publisher {
 public:
  template <typename MessageT>
  void publish(const MessageT& message) const {
    if constexpr (std::is_same_v<MessageT, visualization_msgs::Marker>) {
      if (marker_publisher_) marker_publisher_(message);
    }
  }
  void setMarkerPublisher(std::function<void(const visualization_msgs::Marker&)> publisher) {
    marker_publisher_ = std::move(publisher);
  }
 private:
  std::function<void(const visualization_msgs::Marker&)> marker_publisher_;
};
class Subscriber {};
class NodeHandle {
 public:
  // The ROS2 ReplanFSM adapter fills this bridge from declared ROS2
  // parameters before it calls the untouched planning-manager initialization
  // sequence.  This keeps every original parameter lookup and default in the
  // algorithm source while avoiding a second, divergent configuration path.
  template <typename ValueT>
  void setParam(const std::string& key, ValueT value) {
    parameters_[key] = std::move(value);
  }
  template <typename ValueT>
  void param(const std::string& key, ValueT& value, const ValueT& fallback) const {
    const auto found = parameters_.find(key);
    if (found == parameters_.end()) {
      value = fallback;
      return;
    }
    if (const auto* configured = std::any_cast<ValueT>(&found->second)) {
      value = *configured;
      return;
    }
    value = fallback;
  }
  template <typename MessageT>
  Publisher advertise(const std::string&, int) const { return {}; }
  template <typename MessageT, typename ObjectT>
  Subscriber subscribe(const std::string&, int,
                       void (ObjectT::*)(const std::shared_ptr<const MessageT>&), ObjectT*) const {
    return {};
  }
 private:
  std::unordered_map<std::string, std::any> parameters_;
};
inline void shutdown() {}
}  // namespace ros

#define ROS_INFO(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
#define ROS_WARN(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
#define ROS_ERROR(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
#define ROS_FATAL(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
#define ROS_DEBUG(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
#define ROS_WARN_THROTTLE(period, ...) ROS_WARN(__VA_ARGS__)
#define ROS1_COMPAT_STREAM(level, expression) \
  do { std::ostringstream ros1_compat_stream; ros1_compat_stream << expression; \
       std::fprintf(stderr, "%s\n", ros1_compat_stream.str().c_str()); } while (false)
#define ROS_DEBUG_STREAM(expression) ROS1_COMPAT_STREAM(debug, expression)
#define ROS_INFO_STREAM(expression) ROS1_COMPAT_STREAM(info, expression)
#define ROS_WARN_STREAM(expression) ROS1_COMPAT_STREAM(warn, expression)
#define ROS_ERROR_STREAM(expression) ROS1_COMPAT_STREAM(error, expression)
