# 此文件用于构建、安装和测试 ROS2 双 MID360 点云处理节点。
cmake_minimum_required(VERSION 3.8)
project(livox_cloudpoint_processor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features)

add_library(cloud_algorithms src/cloud_algorithms.cpp)
target_include_directories(cloud_algorithms PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cloud_algorithms nav_msgs pcl_conversions)
target_link_libraries(cloud_algorithms ${PCL_LIBRARIES})

add_executable(livox_cloudpoint_processor_node src/cloud_processor.cpp src/main.cpp)
target_include_directories(livox_cloudpoint_processor_node PRIVATE include)
ament_target_dependencies(livox_cloudpoint_processor_node rclcpp sensor_msgs nav_msgs pcl_ros pcl_conversions livox_ros_driver2)
target_link_libraries(livox_cloudpoint_processor_node cloud_algorithms ${PCL_LIBRARIES})

install(TARGETS cloud_algorithms livox_cloudpoint_processor_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cloud_algorithms test/test_cloud_algorithms.cpp)
  target_link_libraries(test_cloud_algorithms cloud_algorithms ${PCL_LIBRARIES})
  ament_target_dependencies(test_cloud_algorithms nav_msgs)
endif()

ament_package()