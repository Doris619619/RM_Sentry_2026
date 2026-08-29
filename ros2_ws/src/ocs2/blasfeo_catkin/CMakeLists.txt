cmake_minimum_required(VERSION 3.14)
project(blasfeo_catkin)

find_package(ament_cmake REQUIRED)
include(FetchContent)

# Keep the ROS1 wrapper's pinned backend and build settings; only the package
# export mechanism changes for ament/colcon.
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled" FORCE)
FetchContent_Declare(blasfeoDownload
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
  UPDATE_COMMAND "")
FetchContent_MakeAvailable(blasfeoDownload)

install(DIRECTORY ${blasfeodownload_SOURCE_DIR}/include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(blasfeo)
ament_package()
