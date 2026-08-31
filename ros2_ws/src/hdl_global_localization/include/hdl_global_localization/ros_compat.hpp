#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace ros {
class Publisher {
public:
  template <typename MessageT>
  void publish(const MessageT& message) const {
    if (publish_) {
      publish_(&message);
    }
  }

  size_t getNumSubscribers() const {
    return base_ ? base_->get_subscription_count() : 0U;
  }

private:
  friend class NodeHandle;
  std::shared_ptr<rclcpp::PublisherBase> base_;
  std::function<void(const void*)> publish_;
};

class NodeHandle {
public:
  explicit NodeHandle(rclcpp::Node* node) : node_(node) {}

  template <typename ValueT>
  ValueT param(const std::string& name, const ValueT& fallback) const {
    const auto normalized = normalize(name);
    if (!node_->has_parameter(normalized)) {
      // Engines can be replaced at runtime.  Another parameter lookup may have
      // declared the same normalized ROS2 name between this check and declare.
      // In that case the existing value is the authoritative one.
      try {
        node_->declare_parameter<ValueT>(normalized, fallback);
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
      }
    }
    ValueT value = fallback;
    node_->get_parameter_or<ValueT>(normalized, value, fallback);
    return value;
  }

  template <typename MessageT>
  Publisher advertise(const std::string& name, size_t depth, bool transient = false) const {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth));
    if (transient) {
      qos.transient_local();
    }
    auto publisher = node_->create_publisher<MessageT>(name, qos);
    Publisher result;
    result.base_ = publisher;
    result.publish_ = [publisher](const void* message) {
      publisher->publish(*static_cast<const MessageT*>(message));
    };
    return result;
  }

private:
  static std::string normalize(std::string name) {
    std::replace(name.begin(), name.end(), '/', '.');
    return name;
  }

  rclcpp::Node* node_;
};
}  // namespace ros

#define ROS_INFO_STREAM(expression) ((void)0)
#define ROS_WARN_STREAM(expression) ((void)0)
#define ROS_ERROR_STREAM(expression) ((void)0)
