# ROS2 Humble global trajectory planner and its public TrajectoryPoly interface.
cmake_minimum_required(VERSION 3.8)
project(trajectory_generation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoly.msg"
  DEPENDENCIES builtin_interfaces
)

add_library(${PROJECT_NAME}_core src/planner_core.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME}_core PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node PRIVATE ${PROJECT_NAME}_core)
ament_target_dependencies(global_planner_node
  ament_index_cpp geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs
  tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(global_planner_node PRIVATE "${cpp_typesupport_target}")

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz map DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core global_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_link_libraries(test_planner_core ${PROJECT_NAME}_core)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
