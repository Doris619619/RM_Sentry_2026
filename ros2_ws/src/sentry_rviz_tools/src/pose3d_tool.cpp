#include <cmath>

#include <OgrePlane.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/arrow.hpp>

#include "sentry_rviz_tools/pose3d_tool.hpp"

namespace sentry_rviz_tools {
namespace {
bool pointOnGround(rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & point) {
  rviz_rendering::ViewportProjectionFinder finder;
  const auto hit = finder.getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
  if (!hit.first) return false;
  point = hit.second;
  return true;
}
}  // namespace

Pose3DTool::Pose3DTool() = default;
Pose3DTool::~Pose3DTool() { delete arrow_; }

void Pose3DTool::onInitialize() {
  arrow_ = new rviz_rendering::Arrow(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate() {
  setStatus("Left-drag sets XY/yaw; hold right button while dragging to adjust Z.");
  state_ = State::position;
}
void Pose3DTool::deactivate() { if (arrow_) arrow_->getSceneNode()->setVisible(false); }

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  int flags = 0;
  if (event.leftDown()) {
    Ogre::Vector3 intersection;
    if (pointOnGround(event, intersection)) {
      position_ = intersection;
      arrow_->setPosition(position_);
      state_ = State::orientation;
      flags |= Render;
    }
  } else if (event.type == QEvent::MouseMove && event.left()) {
    if (state_ == State::orientation) {
      Ogre::Vector3 current;
      if (pointOnGround(event, current)) {
        yaw_ = std::atan2(current.y - position_.y, current.x - position_.x);
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(yaw_), Ogre::Vector3::UNIT_Z) *
                               Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y));
        if (event.right()) state_ = State::height;
        flags |= Render;
      }
    }
    if (state_ == State::height) {
      position_.z += static_cast<float>(event.wheel_delta) / 120.0f;
      arrow_->setPosition(position_);
      flags |= Render;
    }
  } else if (event.leftUp() && (state_ == State::orientation || state_ == State::height)) {
    onPoseSet(position_.x, position_.y, position_.z, yaw_);
    flags |= Finished | Render;
  }
  return flags;
}
}  // namespace sentry_rviz_tools
