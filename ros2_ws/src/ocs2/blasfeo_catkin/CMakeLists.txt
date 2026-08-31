cmake_minimum_required(VERSION 3.14)
project(blasfeo_catkin)

find_package(ament_cmake REQUIRED)

# Bootstrap owns source acquisition. Build-time networking is forbidden.
set(RM_SENTRY_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor" CACHE PATH
    "Directory populated by third_party.repos")
set(BLASFEO_SOURCE_DIR "${RM_SENTRY_VENDOR_DIR}/blasfeo")
if(NOT EXISTS "${BLASFEO_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "BLASFEO source absent; run ros2_ws/tools/bootstrap_sources.sh first")
endif()

set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled" FORCE)
# HPIPM consumes libblasfeo through this exported package.  Do not exclude the
# vendor target from the default build: an include-only installation leaves a
# broken ament export for downstream isolated colcon packages.
add_subdirectory("${BLASFEO_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/vendor_blasfeo")

install(DIRECTORY "${BLASFEO_SOURCE_DIR}/include/" DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(blasfeo)
ament_package()
