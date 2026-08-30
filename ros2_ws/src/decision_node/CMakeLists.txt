cmake_minimum_required(VERSION 3.8)
project(decision_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(decision_mcu_protocol src/mcu_protocol.cpp src/serial_config.cpp)
target_include_directories(decision_mcu_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(mcu_communicator src/mcu_communicator_node.cpp)
target_link_libraries(mcu_communicator decision_mcu_protocol)
ament_target_dependencies(mcu_communicator geometry_msgs rclcpp std_msgs)

add_executable(strategy_node src/strategy_node.cpp)
target_link_libraries(strategy_node decision_mcu_protocol ${behaviortree_cpp_v3_LIBRARIES})
ament_target_dependencies(strategy_node ament_index_cpp behaviortree_cpp_v3 geometry_msgs rclcpp std_msgs)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(TARGETS decision_mcu_protocol mcu_communicator strategy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_mcu_protocol test/test_mcu_protocol.cpp)
  target_link_libraries(test_mcu_protocol decision_mcu_protocol)
  ament_add_gtest(test_serial_config test/test_serial_config.cpp)
  target_link_libraries(test_serial_config decision_mcu_protocol)
  add_launch_test(test/test_mcu_pty.py TIMEOUT 45)
  add_launch_test(test/test_strategy_scenarios.py TIMEOUT 45)
  add_launch_test(test/test_system_e2e.py TIMEOUT 90)
  add_test(NAME test_central_occupancy_contract COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_central_occupancy_contract.py)
endif()

ament_package()
