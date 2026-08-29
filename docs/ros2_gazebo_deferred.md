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
