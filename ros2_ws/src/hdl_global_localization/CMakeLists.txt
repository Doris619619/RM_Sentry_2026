cmake_minimum_required(VERSION 3.16)
project(hdl_global_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features search registration)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetGlobalMap.srv
  srv/QueryGlobalLocalization.srv
  srv/SetGlobalLocalizationEngine.srv
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs)

add_library(${PROJECT_NAME}_algorithms
  src/hdl_global_localization/bbs/bbs_localization.cpp
  src/hdl_global_localization/ransac/voxelset.cpp
  src/hdl_global_localization/ransac/ransac_pose_estimation.cpp
  src/hdl_global_localization/engines/global_localization_bbs.cpp
  src/hdl_global_localization/engines/global_localization_fpfh_ransac.cpp)
target_include_directories(${PROJECT_NAME}_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_algorithms Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
ament_target_dependencies(${PROJECT_NAME}_algorithms
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs)

add_executable(hdl_global_localization_node src/hdl_global_localization_node.cpp)
target_link_libraries(hdl_global_localization_node ${PROJECT_NAME}_algorithms)
ament_target_dependencies(hdl_global_localization_node
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(hdl_global_localization_node "${cpp_typesupport_target}")

install(TARGETS ${PROJECT_NAME}_algorithms hdl_global_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_engine_defaults test/test_engine_defaults.cpp)
  target_link_libraries(test_engine_defaults ${PROJECT_NAME}_algorithms)
  ament_target_dependencies(test_engine_defaults rclcpp)
  ament_add_pytest_test(test_node_default_modes test/test_node_default_modes.py
    TIMEOUT 30
    ENV PYTHONNOUSERSITE=1
        HDL_GLOBAL_LOCALIZATION_NODE=$<TARGET_FILE:hdl_global_localization_node>)
  add_launch_test(test/test_service_algorithms.py TIMEOUT 90)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_algorithms)
ament_export_dependencies(Eigen3 OpenCV geometry_msgs nav_msgs pcl_conversions PCL rclcpp sensor_msgs std_msgs)
ament_package()
