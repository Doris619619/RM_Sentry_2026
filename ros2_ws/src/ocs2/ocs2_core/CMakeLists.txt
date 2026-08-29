cmake_minimum_required(VERSION 3.8)
project(ocs2_core)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(ocs2_thirdparty REQUIRED)

include(cmake/ocs2_cxx_flags.cmake)
file(GLOB_RECURSE OCS2_CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER OCS2_CORE_SOURCES EXCLUDE REGEX ".*/lintTarget\.cpp$")
add_library(${PROJECT_NAME} SHARED ${OCS2_CORE_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  $<INSTALL_INTERFACE:include>
  ${ocs2_thirdparty_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen Boost::system Boost::filesystem Boost::log_setup Boost::log
  OpenMP::OpenMP_CXX Threads::Threads)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY test/include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 OpenMP Threads ocs2_thirdparty)
ament_package()
