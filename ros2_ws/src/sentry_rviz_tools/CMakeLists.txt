cmake_minimum_required(VERSION 3.16)
project(sentry_rviz_tools)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(${PROJECT_NAME} SHARED
  src/pose3d_tool.cpp
  src/goal3d_tool.cpp
  include/sentry_rviz_tools/pose3d_tool.hpp
  include/sentry_rviz_tools/goal3d_tool.hpp)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs pluginlib rclcpp rviz_common rviz_rendering)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_goal3d_tool test/test_goal3d_tool.cpp)
  target_link_libraries(test_goal3d_tool ${PROJECT_NAME})
  ament_target_dependencies(test_goal3d_tool geometry_msgs pluginlib rclcpp rviz_common)
  find_program(XVFB_RUN xvfb-run REQUIRED)
  add_test(NAME rviz2_goal3d_smoke
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/rviz2_smoke.sh
            ${CMAKE_CURRENT_SOURCE_DIR}/test/goal3d_tool_smoke.rviz)
  set_tests_properties(rviz2_goal3d_smoke PROPERTIES TIMEOUT 20)
endif()

ament_package()
