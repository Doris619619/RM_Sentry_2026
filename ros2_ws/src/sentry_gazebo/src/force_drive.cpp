// Apply bounded forces to a generic omnidirectional rigid chassis; never write pose or velocity state.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace sentry_gazebo {
class ForceDrive final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure, public ignition::gazebo::ISystemPreUpdate {
 public:
  // Explicit generic mass/actuation limits are simulation parameters, not identified real chassis values.
  void Configure(const ignition::gazebo::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
      ignition::gazebo::EntityComponentManager &ecm, ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    link=ignition::gazebo::Link(model.LinkByName(ecm,"base_link"));
    link.EnableVelocityChecks(ecm);
    mass=sdf->Get<double>("mass",15.).first;
    inertia=sdf->Get<double>("yaw_inertia",.65).first;
    maxForce=sdf->Get<double>("max_force",30.).first;
    maxTorque=sdf->Get<double>("max_torque",2.).first;
    transport.Subscribe("/model/sentry/cmd_vel",&ForceDrive::OnCommand,this);
    effort=transport.Advertise<ignition::msgs::Double>("/sim/drive_force");
  }

  // Physics integrates contact and acceleration; the wall watchdog can brake even with all ROS nodes hung.
  void PreUpdate(const ignition::gazebo::UpdateInfo &info, ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused) {
      std::lock_guard<std::mutex> lock(mutex);
      received=0;desired=ignition::math::Vector3d::Zero;filtered=desired;resume=true;
      return;
    }
    const double dt=std::chrono::duration<double>(info.dt).count();
    if (dt<=0 || dt>.1) return;
    ignition::math::Vector3d request;bool expired;
    {
      std::lock_guard<std::mutex> lock(mutex);
      expired=received==0 || Now()-received>500000000 || resume;
      request=expired?ignition::math::Vector3d::Zero:desired;
      resume=false;
    }
    auto delta=request-filtered;
    const double linear=std::hypot(delta.X(),delta.Y()),step=.5*dt;
    if (linear>step) {delta.X()*=step/linear;delta.Y()*=step/linear;}
    delta.Z()=std::clamp(delta.Z(),-.8*dt,.8*dt);
    filtered+=delta;
    // Zero / stale commands request active bounded braking, not the normal acceleration ramp.
    if (expired || request.Length()<1e-9) filtered=ignition::math::Vector3d::Zero;
    const auto pose=link.WorldPose(ecm);const auto worldV=link.WorldLinearVelocity(ecm);
    const auto worldW=link.WorldAngularVelocity(ecm);
    if (!pose || !worldV || !worldW) return;
    const auto localV=pose->Rot().RotateVectorReverse(*worldV);
    const auto localW=pose->Rot().RotateVectorReverse(*worldW);
    // Integral effort balances rolling friction / ramp gravity; freeze integration on saturation.
    ignition::math::Vector3d error(filtered.X()-localV.X(),filtered.Y()-localV.Y(),0);
    auto candidate=integral+error*dt;
    if ((error*20.+candidate*8.).Length()*mass<=maxForce) integral=candidate;
    ignition::math::Vector3d force=mass*(error*20.+integral*8.);
    if (force.Length()>maxForce) force*=maxForce/force.Length();
    if (++counter%5==0) { ignition::msgs::Double value;value.set_data(force.Length());effort.Publish(value); }
    const double yawError=filtered.Z()-localW.Z();
    const double yawCandidate=yawIntegral+yawError*dt;
    if (std::abs(inertia*(12.*yawError+8.*yawCandidate))<=maxTorque) yawIntegral=yawCandidate;
    const double torque=std::clamp(inertia*(12.*yawError+8.*yawIntegral),-maxTorque,maxTorque);
    // Apply drive effort at the model's declared COM; roll/pitch remain free contact dynamics.
    link.AddWorldForce(ecm,pose->Rot().RotateVector(force),ignition::math::Vector3d(0,0,.20));
    link.AddWorldWrench(ecm,ignition::math::Vector3d::Zero,pose->Rot().RotateVector(ignition::math::Vector3d(0,0,torque)));
  }

 private:
  static int64_t Now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  // All six numeric fields must be finite; only planar bounded commands are accepted.
  void OnCommand(const ignition::msgs::Twist &m) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto &v=m.linear(),&w=m.angular();
    if (!(std::isfinite(v.x())&&std::isfinite(v.y())&&std::isfinite(v.z())&&
          std::isfinite(w.x())&&std::isfinite(w.y())&&std::isfinite(w.z()))) {
      desired=ignition::math::Vector3d::Zero;received=0;return;
    }
    double scale=std::max(1.,std::hypot(v.x(),v.y())/.5);
    desired={v.x()/scale,v.y()/scale,std::clamp(w.z(),-.5,.5)};received=Now();
  }
  ignition::gazebo::Link link;
  ignition::transport::Node transport;
  ignition::transport::Node::Publisher effort;
  unsigned counter{0};
  std::mutex mutex;
  ignition::math::Vector3d desired{0,0,0},filtered{0,0,0},integral{0,0,0};
  int64_t received{0};bool resume{false};
  double yawIntegral{0.};
  double mass{15.},inertia{.65},maxForce{30.},maxTorque{2.};
};
}
IGNITION_ADD_PLUGIN(sentry_gazebo::ForceDrive,ignition::gazebo::System,
    sentry_gazebo::ForceDrive::ISystemConfigure,sentry_gazebo::ForceDrive::ISystemPreUpdate)
