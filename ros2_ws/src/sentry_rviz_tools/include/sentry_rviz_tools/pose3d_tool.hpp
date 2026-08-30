#pragma once

#include <rviz_common/tool.hpp>
#include <OgreVector3.h>

namespace rviz_rendering { class Arrow; }

namespace sentry_rviz_tools {
class Pose3DTool : public rviz_common::Tool {
  Q_OBJECT
public:
  Pose3DTool();
  ~Pose3DTool() override;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;
  rviz_rendering::Arrow* arrow_{nullptr};
  Ogre::Vector3 position_;
  double yaw_{0.0};
  enum class State { position, orientation, height } state_{State::position};
};
}  // namespace sentry_rviz_tools
