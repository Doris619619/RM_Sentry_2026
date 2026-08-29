cmake_minimum_required(VERSION 3.8)
project(trajectory_tracking)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_generation REQUIRED)
find_package(sentry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)

add_library(trajectory_tracking_core SHARED
  src/ocs2_sentry/dynamics/SentryRobotDynamtics.cpp
  src/ocs2_sentry/constraint/SentryRobotStateInputConstraint.cpp
  src/ocs2_sentry/constraint/SentryRobotCollisionConstraint.cpp
  src/ocs2_sentry/cost/SentryRobotQuadraticTrackingCost.cpp
  src/KMF.cpp
  src/local_planner.cpp
  src/RM_GridMap.cpp
  src/ocs2_sentry/SentryRobotInterface.cpp)
target_include_directories(trajectory_tracking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(trajectory_tracking_core
  Eigen3::Eigen Boost::filesystem ${OpenCV_LIBS} ${PCL_LIBRARIES}
  ocs2_core::ocs2_core ocs2_oc::ocs2_oc ocs2_mpc::ocs2_mpc ocs2_sqp::ocs2_sqp)
ament_target_dependencies(trajectory_tracking_core rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs pcl_conversions trajectory_generation visualization_msgs tf2_ros tf2_sensor_msgs)
target_compile_features(trajectory_tracking_core PUBLIC cxx_std_14)

if(BUILD_TESTING)
  ament_add_gtest(test_sentry_dynamics test/test_sentry_dynamics.cpp)
  target_link_libraries(test_sentry_dynamics trajectory_tracking_core)
  ament_add_gtest(test_kmf test/test_kmf.cpp)
  target_link_libraries(test_kmf trajectory_tracking_core)
  ament_add_gtest(test_trajectory_poly test/test_trajectory_poly.cpp)
  target_link_libraries(test_trajectory_poly trajectory_tracking_core)
  ament_add_gtest(test_sentry_constraints test/test_sentry_constraints.cpp)
  target_link_libraries(test_sentry_constraints trajectory_tracking_core)
  ament_add_gtest(test_local_planner test/test_local_planner.cpp)
  target_link_libraries(test_local_planner trajectory_tracking_core)
  ament_add_gtest(test_rm_grid_map test/test_rm_grid_map.cpp)
  target_link_libraries(test_rm_grid_map trajectory_tracking_core)
  ament_target_dependencies(test_rm_grid_map ament_index_cpp)
endif()

add_executable(sentry_sqp_benchmark test/sentry_sqp_benchmark.cpp)
target_link_libraries(sentry_sqp_benchmark trajectory_tracking_core)
ament_target_dependencies(sentry_sqp_benchmark ament_index_cpp)

add_executable(hit_bridge src/hit_bridge.cpp)
ament_target_dependencies(hit_bridge rclcpp geometry_msgs std_msgs sentry_msgs)

add_executable(trajectory_tracking_node src/trajectory_tracking_node.cpp)
target_link_libraries(trajectory_tracking_node trajectory_tracking_core)
ament_target_dependencies(trajectory_tracking_node ament_index_cpp rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs trajectory_generation sentry_msgs)

install(TARGETS trajectory_tracking_node hit_bridge sentry_sqp_benchmark
  RUNTIME DESTINATION lib/trajectory_tracking)

install(TARGETS trajectory_tracking_core EXPORT export_trajectory_tracking
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY cfg launch DESTINATION share/trajectory_tracking)

ament_export_targets(export_trajectory_tracking HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs tf2_ros tf2_sensor_msgs Eigen3 Boost OpenCV PCL pcl_conversions trajectory_generation ocs2_core ocs2_oc ocs2_mpc ocs2_sqp)
ament_package()
