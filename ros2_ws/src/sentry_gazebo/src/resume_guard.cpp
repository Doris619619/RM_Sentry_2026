// VelocityControl caches its last velocity across a paused PostUpdate.
// It is loaded before this guard; suppress its cached command for two resume ticks.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>

namespace sentry_gazebo {
class ResumeGuard final : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure,
                          public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 ignition::gazebo::EntityComponentManager &,
                 ignition::gazebo::EventManager &) override {
    model = _entity;
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override {
    if (_info.paused) {
      zeroSteps = 2;
      return;
    }
    if (zeroSteps == 0) return;
    --zeroSteps;
    Zero<ignition::gazebo::components::LinearVelocityCmd>(_ecm);
    Zero<ignition::gazebo::components::AngularVelocityCmd>(_ecm);
  }
 private:
  template<typename Component>
  void Zero(ignition::gazebo::EntityComponentManager &_ecm) {
    const Component zero(ignition::math::Vector3d::Zero);
    auto *component = _ecm.Component<Component>(model);
    if (component) *component = zero;
    else _ecm.CreateComponent(model, zero);
  }
  ignition::gazebo::Entity model{ignition::gazebo::kNullEntity};
  unsigned int zeroSteps{0};
};
}
IGNITION_ADD_PLUGIN(sentry_gazebo::ResumeGuard,
                    ignition::gazebo::System,
                    sentry_gazebo::ResumeGuard::ISystemConfigure,
                    sentry_gazebo::ResumeGuard::ISystemPreUpdate)
