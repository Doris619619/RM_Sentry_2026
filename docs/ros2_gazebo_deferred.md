<!-- Purpose: retain historical Gazebo deferral and link current verified simulation milestones. -->

# Gazebo validation status

As of 2026-09-26, stage-one Fortress GUI and planar motion validation have passed; see [stage-one report](simulation/ACCEPTANCE.md). Stage-two real lidar, exact map geometry, dual-GUI performance and planning visualization are implemented. Following explicit authorization to fix discovered defects, timing/coefficient consistency, acceleration convergence and GUI shutdown ordering have been corrected. All 15 trajectory cases now pass; dual-GUI runtime and repeated clean shutdowns are verified. Automatic tracking remains outside stage two. See the [measured stage-two report](simulation/part2/ACCEPTANCE.md).

## Historical record (superseded environment status)

# ROS2 Gazebo validation status

Status: **Deferred — environment and assets unavailable**. This is a concrete
validation result, not a claim that Gazebo simulation passed.

## Checks performed on the Ubuntu 22.04 / ROS 2 Humble host

- `ros2 pkg list | grep -E "(^gazebo|gazebo_)"` returned no ROS 2 Gazebo
  integration package.
- `command -v gazebo` returned no executable.
- The repository contains no ROS 2 Gazebo package, world (`.world`/`.sdf`), robot
  model, or controller plugin for this Sentry setup.
- The ROS1 sources only contain optional Classic-Gazebo callbacks for
  `/gazebo/model_states` and `/mbot/velodyne_points`; they do not provide a
  portable ROS 2 simulation asset bundle.

## What is validated instead

The ROS2 software simulation launch is independent of Gazebo:

```bash
ros2 launch trajectory_generation global_planning_sim.launch.py use_rviz:=false
```

It was live for 10 seconds without a fatal launch error after declaring its
`waypoint_generator` runtime dependency. The tracking MPC/replanning loop is
covered by the map-frame mocked odometry, point-cloud, trajectory and dynamic
obstacle runtime fixtures documented in `ros2_tracking_validation.md`.

## Required before Level-5 Gazebo validation

1. Install a ROS 2 Humble-compatible Gazebo stack and `gazebo_msgs` equivalent.
2. Supply the Sentry world, robot model, sensor plugins and chassis controller.
3. Define the exact source of `map -> base_link` odometry and map-frame aligned
   point cloud in that simulator.
4. Run a 60-second closed-loop fixture and verify command, stop and replan
   behavior against the same acceptance checks as the software fixture.

MCU serial and physical robot validation remain separate hardware work and were
not attempted.


### 自主仿真第一项验收（2026-09-26）

独立自主入口已通过五组各三次实际 MPC 到点及 90° 初始航向复核；最大到点误差 0.1172 m。
报告与原始证据见 [六项后续仿真验收报告](simulation/autonomy/验收报告.md)。
安全故障注入、动态场景、实际定位、物理底盘与完整通信联调仍分别验收；不宣称六项全部完成。

Autonomy step 2 now passes nine actual fault-injection cases and the verified full-restart CLI.
Native clock rewind is fail-stop and requires a complete restart; it is not supported in place.
See [autonomy acceptance](simulation/autonomy/验收报告.md).

第三项移动中重规划已通过：换目标、真实箱体绕行、箱体位移和封路停车。
实际定位、底盘动力学和完整 MCU 链路仍待第四至六项验收。

第四项实际 CPU HDL/NDT 定位导航已通过往返与故障恢复验收，真值只供评估。
详情见 docs/simulation/autonomy/验收报告.md；第五、六项仍待验收。
