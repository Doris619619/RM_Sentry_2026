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
reported **25 tests, 0 errors, 0 failures, 0 skipped**. The installed
`test_sqp_mpc_smoke` also passed on this master build; its runtime path reaches
HPIPM, and ELF inspection confirmed `libhpipm.so` needs `libblasfeo.so`.

`global_planning_sim.launch.py use_rviz:=false` was kept live for 10 s without a
fatal launch error. Its launch graph requires `waypoint_generator`; that package
is now an explicit `trajectory_generation` runtime dependency so a
`--packages-up-to trajectory_generation` installation contains the executable.

## Deferred scope

Gazebo is deferred and not claimed validated. MCU serial I/O and real-robot
hardware validation are also deferred. See `ros2_gazebo_deferred.md` for the
verified environment and asset gaps.


## 2026-08-30 tracking closure follow-up

Follow-up baseline: Doris `master` `f595b516d50490c84f143cb631b5e5686be000d2`.
The work is on `fix/20260830-tracking-closure`; it does not alter the OCS2
horizon, iteration limit, scheduling, constraints, or HPIPM backend.

### ROS1 Fix45/Fix46 heading contract

The ROS1 source of record is
`HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_tracking/src/tracking_manager.cpp`:
body twist is rotated by the map-frame LiDAR/base yaw before `atan2`; speed above
`0.03 m/s` uses that map-frame velocity heading; otherwise it uses target
 direction/history and never seeds heading from LiDAR yaw. After reference
construction, speed below `0.3 m/s` overrides the MPC observation `phi` with
`ref_phi[0]` and preserves it as history. ROS2 now implements the same sequence
in `trajectory_tracking/tracking_semantics.hpp` and the odometry callback.

`test_tracking_semantics` adds six deterministic cases: stationary yaw/reference
disagreement, `0.1 m/s` low speed, `>0.3 m/s` map-velocity heading, the
stationary-to-low-to-normal transition, missing-reference fallback, and explicit
arrival gating. In particular, `speed=0`, physical yaw=`pi/2`, reference phi=`0`
produces observation phi=`0`, not `pi/2`.

### Arrival and parameter contracts

Tracking now owns `/tracking/arrived`. It publishes `true` only for the existing
position-to-target arrival predicate (distance `<0.3 m`, excluding motion mode
8); new trajectories, no valid trajectory, invalid observations and solver
failures publish `false`. `hit_bridge` still performs only the `SlaverSpeed`
base_link command copy and forwards that explicit state to the compatibility
`/dstar_status`; it no longer infers arrival from zero velocity. This is a
**deliberate behavior change**: a safety stop is no longer reported as an arrival.

`trajectory_tracking/config/tracking.yaml` is the formal node runtime config.
`trajectory_generation/config/map_metadata.yaml` is the shared metadata source
loaded by both Global Planning and Tracking for map resolution/bounds/size,
radius, search ranges, height conversion and replan metadata. The historical
ROS1 tracking launch used `height_bias=-0.38` and second threshold `0.3`, while
current ROS2 Global Planning consumes `0.015294117853045464` and `0.2` for its
occupancy/BEV map contract. Tracking deliberately follows the current ROS2
Global Planning metadata rather than copying the incompatible ROS1 literals.
`task.info` remains the frozen source for OCS2 rollout `0.02 s`, SQP `0.1 s`,
20-point/2.0 s horizon, 8 iterations and HPIPM settings.

The `trajectory_tracking` manifest now declares OpenCV/PCL (`pcl_conversions`,
`pcl_ros`), visualization, tf2, ament-index, ROS messages, local planning and
all direct OCS2 dependencies. `ament_cmake_gtest` is a `test_depend` and its
CMake lookup is inside `BUILD_TESTING`. `rosdep` is not installed on this host,
so `rosdep install` itself could not be executed here; the manifest has been
made complete rather than relying on host-installed libraries.

### Frequency conclusion

ROS1 starts `ros::spin()` and solves from `rcvLidarIMUPosCallback`; ROS2 likewise
solves only in `/localization/odometry`'s subscription callback and has no
control timer. `mpcDesiredFrequency=100` and `mrtDesiredFrequency=400` are
OCS2 task settings, not the node scheduler. The existing 60.053 s fixture
observed 19.016 Hz odometry and 18.817 Hz solver status. No evidence requires a
100 Hz timer, interpolation, horizon or iteration change, so none was made.
The actual vehicle odometry frequency remains a Localization/hardware acceptance
measurement, not a promise encoded by Tracking.

### Follow-up commands and results

- Clean Release dependency closure: `colcon build --packages-up-to
  hdl_localization trajectory_generation trajectory_tracking` completed **14
  packages**; only existing third-party/PCL warnings were emitted.
- `colcon test` for those three packages: **32 tests, 0 errors, 0 failures,
  0 skipped** (including the six new heading/arrival cases).
- Installed `test_sqp_mpc_smoke`: **1/1 passed**, entering real HPIPM/BLASFEO.
- Sentry SQP benchmark (50 warm-ups, 1000 solves): mean **0.104415 ms**, p95
  **0.166486 ms**, max **0.979262 ms**, over 10 ms **0**, CPU **0.129131 s**.
- Real `hit_bridge` topic test: explicit `false` produced
  `/dstar_status=false`; a zero `SlaverSpeed` produced zero `/cmd_vel`; explicit
  `true` produced `/dstar_status=true`.
- Real Tracking invalid-trajectory test (one duration with only three x/y
  coefficients): `solver_status=false`, `/dstar_status=false`, and zero
  `/cmd_vel`.
- Real Tracking arrival test (valid linear trajectory, map pose at its target):
  `/dstar_status=true` and zero `/cmd_vel`.
- `tracking.launch.py` and `global_planning_sim.launch.py use_rviz:=false` both
  remained live for 10 s (timeout status 124) with no fatal/traceback/config
  loading failure.

The prior fixed ROS1/ROS2 command fixture remains valid for yaw=0/reference=0
(`angle_target` absolute difference `0.008612`, `angle_current` `0.003851`, both
below `0.02 m/s`). Low-speed heading is now compared against the current ROS1
Fix45/Fix46 source contract through deterministic regression rather than the old
ROS2 yaw fallback.

Dynamic replanning was rerun after this follow-up: Global Planning and Tracking
were launched together; map odometry was published at `(-3.82, 2.4)`, goal was
`(-1.35, -4.2)`, then map-frame obstacle cloud was published to both
`/aligned_points` and `/filted_topic_3d`. The recorder received **6**
`/global_trajectory` messages and **4** `replan_flag=true` messages. This is a
real `aligned_points -> RM_GridMap -> replan_flag -> ReplanFSM -> new trajectory`
check. Safety/replan behavior remains separate from pure-equivalence claims.
