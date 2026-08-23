# 此文件用于配置 sentry_msgs 的 ROS2 rosidl 消息和服务生成规则。
cmake_minimum_required(VERSION 3.8)
project(sentry_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotsHP.msg"
  "msg/RobotStatus.msg"
  "msg/SlaverSpeed.msg"
  "srv/GoTarget.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
