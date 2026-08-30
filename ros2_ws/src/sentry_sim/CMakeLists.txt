cmake_minimum_required(VERSION 3.16)
project(sentry_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS sentry_sim/fixture_inputs.py DESTINATION lib/${PROJECT_NAME} RENAME fixture_inputs)

if(BUILD_TESTING)
  add_launch_test(test/test_planning_control_fixture.py TIMEOUT 60)
endif()

ament_package()
