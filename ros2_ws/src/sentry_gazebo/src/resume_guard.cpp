// Enforce command freshness inside Gazebo and suppress cached velocity on pause/resume.
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace sentry_gazebo {
class ResumeGuard final : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure,
                          public ignition::gazebo::ISystemPreUpdate {
 public:
  // Own a transport watchdog independent of ROS adapters, bridges and their clocks.
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &,
                 ignition::gazebo::EventManager &) override {
    model = _entity;
    const double configured = _sdf->Get<double>("command_timeout", 0.5).first;
    timeoutNs = static_cast<int64_t>((std::isfinite(configured) && configured > 0 ? configured : 0.5)*1e9);
    const auto topic = _sdf->Get<std::string>("command_topic", "/model/sentry/cmd_vel").first;
    transport.Subscribe(topic, &ResumeGuard::OnCommand, this);
  }

  // Run after VelocityControl: overwrite its model command on expiry even if ROS is frozen.
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override {
    if (_info.dt < std::chrono::steady_clock::duration::zero()) {
      lastCommandNs.store(0,std::memory_order_relaxed);
      zeroSteps=2;
    }
    if (_info.paused) {
      zeroSteps = 2;
      return;
    }
    const auto received = lastCommandNs.load(std::memory_order_relaxed);
    const bool expired = received == 0 || NowNs()-received > timeoutNs;
    if (zeroSteps == 0 && !expired) return;
    if (zeroSteps > 0) --zeroSteps;
    Zero<ignition::gazebo::components::LinearVelocityCmd>(_ecm);
    Zero<ignition::gazebo::components::AngularVelocityCmd>(_ecm);
  }

 private:
  // Monotonic wall time deliberately continues while simulated time stops.
  static int64_t NowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // Invalid transport payloads cannot refresh the actuator watchdog.
  void OnCommand(const ignition::msgs::Twist &message) {
    const auto &v=message.linear(), &w=message.angular();
    const bool finite=std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()) &&
                      std::isfinite(w.x()) && std::isfinite(w.y()) && std::isfinite(w.z());
    lastCommandNs.store(finite ? NowNs() : 0, std::memory_order_relaxed);
  }

  // Replace the desired velocity component without manufacturing state feedback.
  template<typename Component>
  void Zero(ignition::gazebo::EntityComponentManager &_ecm) {
    const Component zero(ignition::math::Vector3d::Zero);
    auto *component = _ecm.Component<Component>(model);
    if (component) *component = zero;
    else _ecm.CreateComponent(model, zero);
  }
  ignition::gazebo::Entity model{ignition::gazebo::kNullEntity};
  ignition::transport::Node transport;
  std::atomic<int64_t> lastCommandNs{0};
  int64_t timeoutNs{500000000};
  unsigned int zeroSteps{0};
};
}
IGNITION_ADD_PLUGIN(sentry_gazebo::ResumeGuard,
                    ignition::gazebo::System,
                    sentry_gazebo::ResumeGuard::ISystemConfigure,
                    sentry_gazebo::ResumeGuard::ISystemPreUpdate)
