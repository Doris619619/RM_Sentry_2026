cmake_minimum_required(VERSION 3.16)
project(sentry_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py
    ENV PYTHONNOUSERSITE=1)
  add_launch_test(test/test_production_safe_mode.py TIMEOUT 45)
  add_launch_test(test/test_production_global_localization.py TIMEOUT 45)
  add_launch_test(test/test_fixture_launch.py TIMEOUT 30)
  add_launch_test(test/test_production_sensor_pipeline.py TIMEOUT 45)
endif()

ament_package()
