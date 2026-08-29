cmake_minimum_required(VERSION 3.8)
project(ocs2_sqp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(blasfeo_catkin REQUIRED)
find_package(hpipm_catkin REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(${PROJECT_NAME} SHARED
  src/SqpLogging.cpp
  src/SqpSettings.cpp
  src/SqpSolver.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ocs2_core::ocs2_core ocs2_oc::ocs2_oc ocs2_mpc::ocs2_mpc
  hpipm_catkin::hpipm_catkin Eigen3::Eigen Boost::system Boost::filesystem)
target_compile_options(${PROJECT_NAME} PUBLIC -pthread -Wfatal-errors -DBOOST_ALL_DYN_LINK)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
if(BUILD_TESTING)
  ament_add_gtest(test_sqp_mpc_smoke test/test_sqp_mpc_smoke.cpp)
  target_link_libraries(test_sqp_mpc_smoke ${PROJECT_NAME})
endif()

add_executable(sqp_mpc_benchmark test/sqp_mpc_benchmark.cpp)
target_link_libraries(sqp_mpc_benchmark ocs2_sqp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 ocs2_core ocs2_oc ocs2_mpc blasfeo_catkin hpipm_catkin)
ament_package()
