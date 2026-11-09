# Build native command/pause protection, orderly RViz and standalone simulation resources.
cmake_minimum_required(VERSION 3.16)
project(sentry_gazebo)
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
add_library(sentry_resume_guard SHARED src/resume_guard.cpp)
target_compile_features(sentry_resume_guard PRIVATE cxx_std_17)
target_link_libraries(sentry_resume_guard ignition-gazebo6::ignition-gazebo6 ignition-transport11::ignition-transport11)
add_library(sentry_force_drive SHARED src/force_drive.cpp)
target_compile_features(sentry_force_drive PRIVATE cxx_std_17)
target_link_libraries(sentry_force_drive ignition-gazebo6::ignition-gazebo6 ignition-transport11::ignition-transport11)
install(TARGETS sentry_resume_guard sentry_force_drive LIBRARY DESTINATION lib)
install(DIRECTORY launch worlds models config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps/sentry_2024 DESTINATION share/${PROJECT_NAME}/maps)
# Expand the size-reduced, pinned upstream arena mesh once during the package build.
set(SENTRY_2024_ASSET_DIR ${CMAKE_CURRENT_BINARY_DIR}/sentry_2024_assets)
set(SENTRY_2024_MESH ${SENTRY_2024_ASSET_DIR}/models/sentry_2024_arena/meshes/base_link.STL)
add_custom_command(OUTPUT ${SENTRY_2024_MESH}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepare_sentry_2024.py --asset-dir ${SENTRY_2024_ASSET_DIR}
  DEPENDS scripts/prepare_sentry_2024.py maps/sentry_2024/source/RMUC2024_field_base_link.STL.gz
    maps/sentry_2024/sources.json maps/sentry_2024/scene.yaml maps/sentry_2024/map_metadata.yaml
    maps/sentry_2024/occ2024low.png maps/sentry_2024/bev2024low.png maps/sentry_2024/occtopo2024low.png
    worlds/planning.sdf worlds/planning_physics.sdf config/autonomy.rviz
  VERBATIM)
add_custom_target(sentry_2024_assets ALL DEPENDS ${SENTRY_2024_MESH})
install(DIRECTORY ${SENTRY_2024_ASSET_DIR}/models/sentry_2024_arena
  ${SENTRY_2024_ASSET_DIR}/models/sentry_2024_arena_visual DESTINATION share/${PROJECT_NAME}/models)
install(FILES ${SENTRY_2024_ASSET_DIR}/worlds/sentry_2024.sdf
  ${SENTRY_2024_ASSET_DIR}/worlds/sentry_2024_physics.sdf DESTINATION share/${PROJECT_NAME}/worlds)
install(FILES ${SENTRY_2024_ASSET_DIR}/config/sentry_2024.rviz DESTINATION share/${PROJECT_NAME}/config)
# Generate the simulation-only NDT reference; production PCD files remain excluded from Git.
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ndt_reference.pcd ${CMAKE_CURRENT_BINARY_DIR}/ndt_reference.json
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ndt_reference_map.py --output ${CMAKE_CURRENT_BINARY_DIR}/ndt_reference.pcd
  DEPENDS scripts/ndt_reference_map.py ../trajectory_generation/map/occfinal.png ../trajectory_generation/config/map_metadata.yaml)
add_custom_target(ndt_reference ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ndt_reference.pcd)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ndt_reference.pcd ${CMAKE_CURRENT_BINARY_DIR}/ndt_reference.json DESTINATION share/${PROJECT_NAME}/maps)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ndt_sentry_2024.pcd ${CMAKE_CURRENT_BINARY_DIR}/ndt_sentry_2024.json
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ndt_reference_map.py
    --output ${CMAKE_CURRENT_BINARY_DIR}/ndt_sentry_2024.pcd
    --map-directory ${CMAKE_CURRENT_SOURCE_DIR}/maps/sentry_2024
    --metadata ${CMAKE_CURRENT_SOURCE_DIR}/maps/sentry_2024/map_metadata.yaml
  DEPENDS scripts/ndt_reference_map.py maps/sentry_2024/map_manifest.json
    maps/sentry_2024/occ2024low.png maps/sentry_2024/map_metadata.yaml)
add_custom_target(ndt_sentry_2024 ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/ndt_sentry_2024.pcd)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ndt_sentry_2024.pcd ${CMAKE_CURRENT_BINARY_DIR}/ndt_sentry_2024.json DESTINATION share/${PROJECT_NAME}/maps)
install(PROGRAMS scripts/pty_mcu_sim.py scripts/mcu_wire.py scripts/decision_goal_router.py scripts/estimated_odometry.py scripts/sim_adapter.py scripts/keyboard.py scripts/cloud_adapter.py scripts/map_display.py scripts/autonomy_guard.py scripts/autonomy_geometry.py scripts/gui_layout.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/simulation_map.py DESTINATION lib/${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_executable(rviz_safe src/rviz_safe.cpp)
target_compile_features(rviz_safe PRIVATE cxx_std_17)
ament_target_dependencies(rviz_safe rclcpp rviz_common)
target_link_libraries(rviz_safe Qt5::Widgets)
install(TARGETS rviz_safe RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()
