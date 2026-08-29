# ROS2 trajectory tracking validation evidence

Baseline: `de7c798f9dc3c448b4500fe4afef5362ac6e8312` from Doris `master`.
All ROS2 commands use Humble on Ubuntu 22.04 and Release builds. No ROS1 runtime
or `ros1_bridge` is part of the ROS2 runtime; Noetic below is only a fixture comparator.

## Build and unit coverage

Release `colcon build` and `colcon test --packages-select trajectory_tracking`
completed successfully: **16 assertions, 0 errors, 0 failures, 0 skipped**.
Coverage includes dynamics/Jacobian, KMF, polynomial order/invalid input,
state/input and collision constraints, LocalPlanner lead clamp/mode-8 evade, and
RM_GridMap map-frame cloud ingestion.

## ROS1/ROS2 fixed fixture

Fixture: map-frame odometry `(0, 0, yaw=0)`, zero body twist, a 20 s linear
trajectory encoded in ROS1 order `[cubic, quadratic, linear, constant] = [0, 0,
1, 0]`, and identical map/task/HPIPM pins.

| output | ROS1 Noetic | ROS2 Humble | absolute difference |
| --- | ---: | ---: | ---: |
| `SlaverSpeed.line_speed` | 0.000000 | 0.000000 | 0.000000 |
| `angle_target` | -0.332676 | -0.341287 | 0.008612 |
| `angle_current` | -0.001847 | -0.005699 | 0.003851 |
| `solver_status` | true | true | exact |

The command components meet the 0.02 m/s fixture tolerance. `line_speed` remains
the ROS1 compatibility field; actual base_link `vx`/`vy` are
`angle_target`/`angle_current`, copied by `hit_bridge` to `/cmd_vel.linear.x/.y`.

## ROS2 adapter runtime checks

- A lidar-frame `/aligned_points` message was accepted through an explicit
  identity `map -> lidar` tf2 transform. No missing-TF or malformed-cloud
  diagnostic was emitted. A cloud without that transform is dropped rather than
  being inserted into the map-frame grid.
- Real node output contained nonempty `map`-frame
  `/tracking/mpc_predicted_path` (5,961 bytes) and
  `/tracking/mpc_reference_path` (4,668 bytes), plus
  `/robot_cur_yaw_reg=0.0` for the zero-yaw fixture.

## Safety and replanning runtime checks

- A malformed trajectory with three coefficients for one duration produced
  `solver_status=false`, zero `SlaverSpeed`, and zero `/cmd_vel` via real
  `hit_bridge`.
- A map-frame point cloud placed on the planned path produced 41 consecutive
  predicted obstacle samples at 3.02 s. Tracking published `/replan_flag`; real
  ReplanFSM then emitted new trajectories. The recorder observed three valid
  `/global_trajectory` messages (initial plus replan updates).

## Release performance

Real Sentry problem, 20 samples/2.0 s horizon, two SQP threads, eight maximum
iterations, HPIPM `ROBUST`, `reg_prim=1e-8`, 50 warm-ups and 1000 measurements:

| metric | result |
| --- | ---: |
| mean solve time | 0.086072 ms |
| p95 solve time | 0.134222 ms |
| max solve time | 0.755594 ms |
| solves over 10 ms | 0 |
| process CPU time | 0.100438 s |

A 60.053 s ROS2 node fixture published 1,142 odometry samples (19.016 Hz). It
received 1,130 solver-status messages (18.817 Hz), with output interval p95
54.452 ms and max 106.192 ms; tracking-node CPU time was 3.120 s. Thus
`mpcDesiredFrequency=100` is not an observed runtime frequency. This branch does
not change scheduling, horizon, iteration limit, constraints, or QP backend to
hide that miss.

## Master post-merge verification

On Doris `master` at `81aa443`, a clean Release `--packages-up-to` build of the
localization, global-planning, OCS2/HPIPM/BLASFEO and tracking closure completed
all 13 packages. The three primary packages
`hdl_localization`, `trajectory_generation`, and `trajectory_tracking` then
reported **25 tests, 0 errors, 0 failures, 0 skipped**.

`global_planning_sim.launch.py use_rviz:=false` was kept live for 10 s without a
fatal launch error. Its launch graph requires `waypoint_generator`; that package
is now an explicit `trajectory_generation` runtime dependency so a
`--packages-up-to trajectory_generation` installation contains the executable.

## Deferred scope

Gazebo is deferred and not claimed validated. MCU serial I/O and real-robot
hardware validation are also deferred. See `ros2_gazebo_deferred.md` for the
verified environment and asset gaps.
