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


## 当前自主仿真状态

六项自主导航仿真已经分别验收通过：实际 MPC 闭环、异常停车与接管、
真实动态障碍重规划、实际 CPU NDT、通用接触动力学，以及真实策略和 MCU 可执行程序的 PTY 完整链路。
完整栈双 GUI 310 秒平均 RTF 0.853、RSS 峰值 2.05 GiB；完整重启关键发布者 1 → 0 → 1，
旧子进程无残留，重启一辆车且不执行旧目标。

上文 Deferred 是历史环境记录，已不代表当前 Gazebo 能力。
真实电控硬件、真实轮系辨识、Point-LIO、未知位置重定位和完整比赛行为仍需单独验收。
原生时间回拨必须完整重启，不能原地恢复。

[六项实测与失败记录](simulation/autonomy/验收报告.md) ·
[完整接口与命令](simulation/autonomy/完整系统接口.md) ·
[队会汇报和交接](simulation/autonomy/队会汇报与交接.md)。
