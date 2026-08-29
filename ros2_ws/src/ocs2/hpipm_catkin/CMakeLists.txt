cmake_minimum_required(VERSION 3.14)
project(hpipm_catkin)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(blasfeo_catkin REQUIRED)
include(FetchContent)

# hpipm's CMake consumes an installed BLASFEO prefix.  In an isolated colcon
# build the dependency prefix is the grandparent of its package CMake directory.
get_filename_component(BLASFEO_PREFIX "${blasfeo_catkin_DIR}/../../.." ABSOLUTE)
set(BLASFEO_PATH "${BLASFEO_PREFIX}" CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR "${BLASFEO_PATH}/include" CACHE STRING "BLASFEO headers" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled" FORCE)
FetchContent_Declare(hpipmDownload
  GIT_REPOSITORY https://github.com/giaf/hpipm
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
  UPDATE_COMMAND "")
FetchContent_MakeAvailable(hpipmDownload)

add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${hpipmdownload_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ocs2_core::ocs2_core ocs2_oc::ocs2_oc Eigen3::Eigen PRIVATE hpipm)
target_compile_options(${PROJECT_NAME} PUBLIC -pthread -Wfatal-errors -DBOOST_ALL_DYN_LINK)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${hpipmdownload_SOURCE_DIR}/include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 ocs2_core ocs2_oc blasfeo_catkin)
ament_package()
