cmake_minimum_required(VERSION 3.8)
project(ocs2_mpc)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
file(GLOB_RECURSE OCS2_MPC_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER OCS2_MPC_SOURCES EXCLUDE REGEX ".*/lintTarget\.cpp$")
add_library(${PROJECT_NAME} SHARED ${OCS2_MPC_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ocs2_core::ocs2_core ocs2_oc::ocs2_oc Eigen3::Eigen
  Boost::system Boost::filesystem Boost::log_setup Boost::log)
target_compile_options(${PROJECT_NAME} PUBLIC -pthread -Wfatal-errors -DBOOST_ALL_DYN_LINK)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 ocs2_core ocs2_oc)
ament_package()
