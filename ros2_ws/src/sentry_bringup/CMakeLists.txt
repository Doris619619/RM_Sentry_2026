cmake_minimum_required(VERSION 3.16)
project(sentry_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py
    ENV PYTHONNOUSERSITE=1)
endif()

ament_package()
