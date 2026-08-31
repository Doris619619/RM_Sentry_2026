# 此文件用于构建 ROS2 HDL 定位节点、地图服务器及其消息接口。
cmake_minimum_required(VERSION 3.16)
project(hdl_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(hdl_global_localization REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(Eigen3 REQUIRED)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ScanMatchingStatus.msg
  msg/HdlRelocStatus.msg
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_lib SHARED
  src/hdl_localization/pose_estimator.cpp
  apps/hdl_localization_nodelet.cpp
  apps/globalmap_server_nodelet.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  ndt_omp
  hdl_global_localization
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}"
  ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib OpenMP::OpenMP_CXX)
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "hdl_localization::HdlLocalizationNodelet"
  EXECUTABLE ${PROJECT_NAME}_node
)
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "hdl_localization::GlobalmapServerNodelet"
  EXECUTABLE ${PROJECT_NAME}_map_server
)

add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_map_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_tf_contract test/test_tf_contract.cpp)
  ament_add_gtest(test_odometry_twist_contract test/test_odometry_twist_contract.cpp)
  if(TARGET test_odometry_twist_contract)
    ament_target_dependencies(test_odometry_twist_contract nav_msgs geometry_msgs)
    target_link_libraries(test_odometry_twist_contract Eigen3::Eigen)
    target_include_directories(test_odometry_twist_contract PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  if(TARGET test_tf_contract)
    target_link_libraries(test_tf_contract Eigen3::Eigen)
    target_include_directories(test_tf_contract PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
  add_launch_test(test/test_relocalize_integration.py TIMEOUT 90)
endif()

ament_package()
