# 此文件用于生成 HDL 定位节点可选全局重定位服务的 ROS2 接口。
cmake_minimum_required(VERSION 3.16)
project(hdl_global_localization)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetGlobalMap.srv
  srv/QueryGlobalLocalization.srv
  srv/SetGlobalLocalizationEngine.srv
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
