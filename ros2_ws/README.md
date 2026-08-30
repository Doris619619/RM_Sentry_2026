# RM Sentry ROS 2 Humble workspace

`ros2_ws` is the ROS 2 production workspace for Ubuntu 22.04 + ROS 2 Humble.  The
ROS 1 workspaces in this repository are legacy audit sources, not build or runtime
entry points.

## Reproducible setup

The only supported setup path is:

```bash
cd /home/liangys/RM_Sentry_2026/ros2_ws
./tools/bootstrap_sources.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

The bootstrap script imports the pinned sources in `third_party.repos`, builds the
repository's existing `../Livox-SDK2` into `.deps/livox-sdk2`, and runs rosdep.
It does not use `sudo`, `ldconfig`, or a second Livox SDK checkout.  It may use
normal proxy environment variables only while importing the pinned sources.
After bootstrap completes, `colcon build` must not fetch source code.

Build with the workspace SDK prefix visible to CMake:

```bash
export CMAKE_PREFIX_PATH="$PWD/.deps/livox-sdk2:${CMAKE_PREFIX_PATH}"
colcon build --symlink-install
colcon test
colcon test-result --all --verbose
```

## Supported entry points

Production composition is explicit and safety-closed by default:

```bash
ros2 launch sentry_bringup production.launch.py \
  globalmap_pcd:=/absolute/path/to/competition_map.pcd \
  enable_sensor_pipeline:=true
```

When `enable_sensor_pipeline:=true`, it starts the localization/control chain
`Livox + IMU -> cloud processing -> Point-LIO -> HDL localization -> planning ->
tracking -> /cmd_vel`, and the connected decision loop
`MCU/referee/radar -> Decision -> /clicked_point -> planning -> tracking ->
/tracking/arrived,/dstar_status -> Decision`.

`enable_sensor_pipeline`, `enable_mcu`, and `allow_motion_output` all default to `false`.
The first gate protects physical Livox/Point-LIO startup; the latter two protect the chassis path.  A real serial
MCU is opened only when both are explicitly enabled.  The normal serial contract
is `921600`; `115200` is only for a deliberately selected compatibility test.

For deterministic software fixtures (not hardware or physics validation):

```bash
ros2 launch sentry_bringup fixture.launch.py
```

The fixture publishes odometry, filtered cloud, Livox `CustomMsg`, IMU and
referee/radar inputs.  It does not model a robot, chassis, sensors, or a world.
The existing Decision PTY E2E test remains the required decision-loop contract.

`run_planning_sim.sh` and `send_goal.sh` at repository root are ROS 1/Noetic
legacy scripts.  Do not use them for ROS 2 deployment.

## Scope and hardware boundary

`hdl_global_localization` is part of this workspace: normal launch loads BBS as
its effective default; a node started without parameter configuration falls back
to FPFH_RANSAC.  TEASER is intentionally unavailable in this production build and
must not download code.

This workspace does not claim ROS 2 Gazebo physics simulation, ROS 2
`hdl_graph_slam` offline mapping, FAST_LIO, or LiDAR_IMU_Init migration.  PCD map
delivery is the controlled replacement for offline graph-SLAM in the runtime
chain.  Successful builds and fixtures are not hardware acceptance: real
MID360/IMU, MCU/referee/radar, chassis safety, map convergence and long-running
vehicle tests remain mandatory.
