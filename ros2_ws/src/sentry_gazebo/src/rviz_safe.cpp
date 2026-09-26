// Run standard RViz displays while handling termination on the Qt thread before ROS teardown.
#include <csignal>
#include <memory>
#include <string>
#include <vector>
#include <QApplication>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualizer_app.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;

// A POSIX handler may only set the flag; Qt and ROS cleanup run on the GUI thread.
void requestStop(int) { stop_requested = 1; }

// Stop render timers before VisualizerApp tears down ROS subscriptions and Ogre resources.
void stopRendering(QApplication& app) {
  for (QWidget* widget : app.topLevelWidgets()) {
    auto* frame = qobject_cast<rviz_common::VisualizationFrame*>(widget);
    if (frame) {
      frame->getManager()->stopUpdate();
      frame->setWindowModified(false);
    }
  }
  app.quit();
}
}

// Use installed RViz plugins/configuration, changing only signal-to-shutdown ordering.
int main(int argc, char** argv) {
  auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> qt_arguments;
  for (auto& argument : arguments) qt_arguments.push_back(argument.data());
  int qt_argc = static_cast<int>(qt_arguments.size());
  QApplication app(qt_argc, qt_arguments.data());
  rviz_common::VisualizerApp visualizer(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  visualizer.setApp(&app);
  if (!visualizer.init(argc, argv)) return 1;

  // The default asynchronous ROS handler invalidates the context while a frame
  // can still render. Let the main Qt loop finish first, then the RViz destructor
  // performs its normal ROS/Ogre cleanup. Crashes retain their original status.
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  QTimer signal_timer;
  // Polling stays in the Qt thread and adds at most 20 ms to orderly shutdown.
  QObject::connect(&signal_timer, &QTimer::timeout, [&app]() {
    if (stop_requested) stopRendering(app);
  });
  signal_timer.start(20);
  return app.exec();
}
