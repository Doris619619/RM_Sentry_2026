cmake_minimum_required(VERSION 3.8)
project(ocs2_oc)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ocs2_core REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
file(GLOB_RECURSE OCS2_OC_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER OCS2_OC_SOURCES EXCLUDE REGEX ".*/lintTarget\.cpp$")
add_library(${PROJECT_NAME} SHARED ${OCS2_OC_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ocs2_core::ocs2_core Eigen3::Eigen Boost::system Boost::filesystem)
target_compile_options(${PROJECT_NAME} PUBLIC -pthread -Wfatal-errors -DBOOST_ALL_DYN_LINK)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY test/include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 ocs2_core)
ament_package()
