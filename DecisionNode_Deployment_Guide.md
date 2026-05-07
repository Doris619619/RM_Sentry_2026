# DecisionNode Deployment Guide For Old_nav

## Purpose

This guide describes the practical deployment sequence for running the autonomous robot with the old_nav stack plus DecisionNode.

It assumes you already have the required map assets prepared:

- a 2D occupancy map: `.pgm` plus `.yaml`
- a 3D localization map: `.pcd`
- a valid set of decision target points for actions like `SUPPLY`, `WAITFOROP`, `RADICAL`, and `OCCUPY`

The important architectural split is:

- old_nav is responsible for map serving, localization, TF alignment, path planning, and velocity generation
- DecisionNode is responsible for tactical state selection and publishing goal points and support flags

In deployment terms, DecisionNode should be treated as the layer that decides `where to go next`, while old_nav decides `how to get there`.

## What Each Asset Is Used For

### 1. PGM + YAML

These are used by `map_server` for the 2D planning map.

Relevant files:

- `Old_nav/sim_nav/src/bot_sim/launch_real/map_server.launch`
- `Old_nav/sim_nav/src/bot_sim/map/base.yaml`

The `.yaml` points to the `.pgm`, and also defines:

- `resolution`
- `origin`
- occupancy thresholds

This map is what the D* Lite planner sees on `/map`.

### 2. PCD

This is used by `hdl_localization` as the global point cloud map for scan matching and pose initialization.

Relevant file:

- `Old_nav/sim_nav/src/hdl_localization/launch/hdl_localization.launch`

The parameter currently used is:

```xml
<param name="globalmap_pcd" value="$(find hdl_graph_slam)/map/base.pcd" />
```

So if your deployable map is not `base.pcd`, you must update that launch file or provide an override.

### 3. Decision Goal Points

These are the coordinates that DecisionNode publishes to `/clicked_point` depending on the current behavior tree state.

Relevant file:

- `DecisionNode/src/decision_node/launch/strategy_decision_tmp.launch`

Those points are not produced by localization or planning. They are fixed strategic anchors that you define before deployment.

## The Core Runtime Dataflow

At runtime, the important chain is:

1. `map_server` publishes the 2D map.
2. LiDAR plus IMU feed `hdl_localization`.
3. `hdl_localization` estimates the robot pose in the map frame.
4. TF utilities convert that pose into the frames old_nav uses for planning.
5. `dstarlite` waits for a goal on `/clicked_point`.
6. `strategy_node` decides the current action and publishes a `geometry_msgs/PointStamped` goal to `/clicked_point`.
7. `dstarlite` plans toward that point and generates motion commands.
8. `/dstar_status` feeds back arrival state to DecisionNode.
9. DecisionNode may change state and publish the next goal.

That means your deployment work is mostly about making sure all maps, transforms, initial pose, and decision points live in the same coordinate system.

## Files That Matter Most Before Deployment

### old_nav launch chain

- `Old_nav/3DNavUL_Test.launch`
- `Old_nav/3DNavUL_Test_with_decision.launch`
- `Old_nav/run_3DNavUL_Test_with_decision.sh`

### old_nav map and localization

- `Old_nav/sim_nav/src/bot_sim/launch_real/map_server.launch`
- `Old_nav/sim_nav/src/bot_sim/map/base.yaml`
- `Old_nav/sim_nav/src/hdl_localization/launch/hdl_localization.launch`

### DecisionNode behavior and points

- `DecisionNode/src/decision_node/launch/strategy_decision_tmp.launch`
- `DecisionNode/src/decision_node/config/strategy_tree.xml`
- `DecisionNode/src/decision_node/src/strategy_node.cpp`

### MCU and bridge behavior

- `DecisionNode/src/decision_node/launch/mcu_communicator.launch`
- `DecisionNode/src/decision_node/src/mcu_communicator.cpp`
- `Old_nav/sim_nav/src/bot_sim/launch_real/ser2msg_tf_decision_givepoint.launch`
- `Old_nav/sim_nav/src/bot_sim/src/ser2msg_decision_givepoint.cpp`

## Recommended Deployment Preparation Sequence

Do this in order. Do not start with the full combined launch before each layer is validated on its own.

### Step 1. Freeze the map package for this venue

For each deployable field or venue, keep one coherent map bundle:

- one `.pgm`
- one `.yaml`
- one `.pcd`
- one matching set of tactical points

Recommended rule: treat these four as one versioned package.

If the `.pgm/.yaml` and `.pcd` came from different map builds or different origins, your deployment will look correct in pieces but fail in behavior because localization and planning will disagree about where things are.

### Step 2. Make the 2D planning map correct first

Confirm that `map_server` is loading the correct YAML.

Current old_nav launch uses:

```xml
<arg name="map" default="base.yaml" />
<node name="map_server" pkg="map_server" type="map_server" args="$(find bot_sim)/map/$(arg map)"/>
```

That means:

- if you want to deploy a new occupancy map, either replace `base.yaml/base.pgm`
- or change the launch argument so the desired yaml is loaded explicitly

Deployment check:

- map opens in RViz
- robot pose lands in the correct place on the 2D map
- walls and traversable space match reality

### Step 3. Make the 3D localization map correct second

Confirm `hdl_localization.launch` points to the correct `.pcd`.

You must verify:

- the `.pcd` corresponds to the same physical environment as the `.pgm/.yaml`
- the map frame origin is consistent with the 2D occupancy map
- the initial pose values are close enough for localization to converge

The current launch hardcodes an initial pose:

```xml
<param name="specify_init_pose" value="true" />
<param name="init_pos_x" value="6.798271894454956" />
<param name="init_pos_y" value="-1.80238938331604" />
<param name="init_ori_w" value="0.7017" />
<param name="init_ori_z" value="-0.7017" />
```

Before field deployment, update these to your actual startup pose if the robot does not always start from the same spot.

If you do not, the entire stack may launch successfully but localization may settle into the wrong part of the map.

### Step 4. Validate old_nav without DecisionNode

Before enabling autonomy, launch the pure old_nav stack and manually publish goals to `/clicked_point`.

Goal of this step:

- prove localization is stable
- prove `dstarlite` can plan from the current pose
- prove `/cmd_vel` and downstream motion are correct
- prove `/dstar_status` changes when the goal is reached

If old_nav cannot drive correctly from a manually sent `/clicked_point`, DecisionNode will not save you. It only automates goal selection.

### Step 5. Define the tactical points DecisionNode will use

This is the part you asked about directly.

Before deployment, you should assign map coordinates to each strategic state in `strategy_decision_tmp.launch`.

Current namespaces and their meaning are:

- `occupy`: cyclic patrol or occupy points used by `PUSH` and `OCCUPY`
- `supply`: supply or refill point
- `waitforop`: default waiting or staging point
- `retreat`: fallback point used under intense harm while in `WAITFOROP`
- `radical`: aggressive push point
- `radical1`: follow-on aggressive point

These are loaded through parameters like:

```xml
<param name="/goals/occupy/point_0/x" value="..."/>
<param name="/goals/occupy/point_0/y" value="..."/>
<param name="/goals/supply/x" value="..."/>
<param name="/goals/waitforop/x" value="..."/>
<param name="/goals/retreat/x" value="..."/>
<param name="/goals/radical/x" value="..."/>
<param name="/goals/radical1/x" value="..."/>
```

### Step 6. Choose the points by tactical intent, not just geometry

Use the following logic when setting coordinates.

## How To Set A Valid Decision Target Point Set

This is the practical workflow I would use in this repo.

### 1. Pick points in the `map` frame only

DecisionNode publishes `geometry_msgs/PointStamped` goals with `frame_id="map"`, and old_nav's D* Lite consumes `/clicked_point` as a map-frame goal.

That means every decision point you store in `strategy_decision_tmp.launch` must be a 2D coordinate in the same map frame as:

- the `.yaml/.pgm` occupancy map
- the `.pcd` localization map
- the live localized robot pose in RViz

If you choose points visually but in the wrong frame, the stack will still run but the robot will drive to the wrong physical place.

### 2. Do not set points by editing numbers first

The correct order is:

1. bring up old_nav until localization is healthy
2. manually test candidate goals on `/clicked_point`
3. keep only the points that plan and execute cleanly
4. write those coordinates into DecisionNode

Do not start from the launch file and guess coordinates by hand unless you already know the map very well.

### 3. Use RViz or `/clicked_point` to probe candidate locations

Because you already know old_nav, the fastest way is to validate each candidate as a normal nav goal first.

Recommended method:

1. Launch old_nav without relying on DecisionNode action switching.
2. In RViz, use `Publish Point` on the map.
3. Click the candidate location.
4. Watch whether D* Lite produces a stable path and whether the robot reaches it cleanly.
5. Record the final accepted coordinate.

You can also publish directly from the terminal if needed:

```bash
rostopic pub -1 /clicked_point geometry_msgs/PointStamped '{header: {frame_id: "map"}, point: {x: 3.9, y: -1.7, z: 0.0}}'
```

For each candidate, check:

- D* can generate a path immediately
- the path does not graze inflated walls or corners
- the robot can physically settle there without oscillation
- `/dstar_status` eventually reports arrival
- localization remains stable near that point

If any of those fail, the point is not deployment-valid yet.

### 4. Apply a hard validity filter before accepting a point

For this stack, a point is valid only if all of these are true:

- it lies in free space on the 2D occupancy map
- it has margin from walls, pillars, and inflated obstacles
- D* reaches it from the expected starting side, not only from one lucky approach
- the robot can stop there without getting trapped in a narrow passage
- it still makes tactical sense for the action that uses it

As a practical rule, do not place points:

- on obstacle boundaries
- inside narrow doorways or choke centers
- exactly on the supply structure or wall edge
- at a place that requires perfect localization to be safe

Choose the center of a usable free-space patch, not the geometric edge of the feature you care about.

### 5. Choose each action point by role

#### `SUPPLY`

Use one fixed point.

Good `SUPPLY` points are:

- the easiest legal supply-access point for the robot to reach
- slightly offset into free space rather than on the exact supply structure
- robust under low HP, low ammo, and degraded local conditions

Bad `SUPPLY` points are:

- too deep into a corner
- too close to a wall or static obstacle
- only reachable from one direction

#### `WAITFOROP`

Use one fixed staging point.

Good `WAITFOROP` points are:

- safe while idle
- fast to exit from
- good for transitioning either to `PUSH` or to `RETREAT`

This should be a low-risk parking or staging anchor, not just any open cell.

#### `RETREAT`

Use one fixed fallback point.

In the current tree, `WAITFOROP` can switch to `RETREAT` under `IntenseHarm`, so `RETREAT` should be meaningfully safer than `WAITFOROP`, not just nearby.

Good `RETREAT` points are:

- deeper behind cover
- easier to localize in
- reachable quickly from the usual combat corridor

#### `RADICAL`

Use one fixed aggressive point.

This is your first offensive anchor. It should be:

- farther forward than `WAITFOROP`
- tactically useful under score advantage
- still reachable without repeated replanning failures

If D* consistently gives unstable paths to a `RADICAL` point, move the point back into cleaner free space. Do not try to fix that with BT thresholds.

#### `RADICAL1`

Use one fixed follow-up aggressive point.

In the current tree, `RADICAL1` is the next step after `RADICAL` when advantage remains favorable and arrival is detected. That means `RADICAL1` should be a deeper or later-stage pressure point, not just a duplicate of `RADICAL`.

#### `OCCUPY`

Treat these as a route, not as isolated markers.

In the current tree:

- `PUSH` uses `SetGoalFromParamsCyclic ns="occupy" point_count="2"`
- `OCCUPY` uses the same cyclic list with `point_count="2"`

So right now only `occupy_point_0` and `occupy_point_1` matter.

Use them like this:

- `occupy_point_0`: approach or entry point into the occupy area
- `occupy_point_1`: hold or pressure point after the first arrival

Only define `occupy_point_2` and `occupy_point_3` if you plan to edit the tree to `point_count="4"`.

### 6. Record accepted points into `strategy_decision_tmp.launch`

Once a candidate point has passed manual nav validation, write it into:

- `/home/sentry/AstarTraining/DecisionNode/src/decision_node/launch/strategy_decision_tmp.launch`

The fields you edit are the launch args near the top:

```xml
<arg name="supply_x" default="..."/>
<arg name="supply_y" default="..."/>
<arg name="waitforop_x" default="..."/>
<arg name="waitforop_y" default="..."/>
<arg name="retreat_x" default="..."/>
<arg name="retreat_y" default="..."/>
<arg name="radical_x" default="..."/>
<arg name="radical_y" default="..."/>
<arg name="radical1_x" default="..."/>
<arg name="radical1_y" default="..."/>
<arg name="occupy_point_0_x" default="..."/>
<arg name="occupy_point_0_y" default="..."/>
<arg name="occupy_point_1_x" default="..."/>
<arg name="occupy_point_1_y" default="..."/>
```

Those args are then copied onto the parameter server under `/goals/...`, which is what the behavior tree reads.

### 7. Prefer field-specific profiles over ad hoc edits

For real deployment, do not keep retyping coordinates into the same file before each match.

Better options are:

1. keep one launch file per field or side
2. keep one wrapper launch that overrides the args per venue
3. keep a documented table of approved points for each map package

This repo already has backup variants such as `strategy_decision_tmp.launch.red.backup` and `strategy_decision_tmp.launch.blue.backup`, which is a sign that field-specific or side-specific point sets are already part of the intended workflow.

### 8. Re-validate after writing the points

After the coordinates are in the launch file, bring up the combined stack and verify the actual DecisionNode outputs:

```bash
rostopic echo /clicked_point
```

Then force or wait for each action and confirm the published goal matches the intended coordinate.

For `OCCUPY`, make sure the system alternates only between `point_0` and `point_1` unless you have explicitly changed the tree.

### 9. Use this acceptance test for every final point set

A decision target set is deployment-ready only if:

1. every single-point action publishes the expected coordinate
2. every point is reachable from the expected operating region
3. no point causes repeated planner oscillation or wall-hugging
4. `SUPPLY` and `RETREAT` remain safe under stress conditions
5. the `OCCUPY` sequence matches the route you actually want to patrol
6. the point list matches the `point_count` in the tree

If any one of those is false, the point set is not ready.

#### `SUPPLY`

This should be:

- reachable with high certainty
- low risk
- aligned with actual refill mechanics and legal field behavior

Do not place it at the exact edge of obstacles or in a narrow passage. Treat it as a high-confidence recovery anchor.

#### `WAITFOROP`

This is your idle or staging point.

Choose a location that:

- keeps the robot safe before game start or while waiting for a favorable condition
- preserves line of sight or transition speed into the next task
- does not block other assets

#### `RETREAT`

This is not just a backup copy of `WAITFOROP`.

Choose a point that is:

- easier to defend
- easier to localize in
- lower risk under active damage

If the robot takes sustained harm, this point should reduce exposure immediately.

#### `RADICAL` and `RADICAL1`

These should be aggressive but still navigable.

Recommended pattern:

- `RADICAL`: entry or pressure point
- `RADICAL1`: deeper follow-up point after advantage is confirmed

Do not place them where D* regularly produces unstable or obstacle-hugging paths unless that is an intentional tradeoff.

#### `OCCUPY`

These are the most important points to think through carefully.

In the current tree, both `PUSH` and `OCCUPY` use the `occupy` cyclic points.

That means the `occupy` list is effectively your patrol or contest route, not just one single central target.

The right way to choose these points is:

- point 0: first approach point with high reachability
- point 1: second pressure or hold point
- point 2 and point 3: only if you actually enable four-point cycling in the tree

Important current detail:

The launch file defines four occupy points, but the behavior tree currently uses `point_count="2"` in the `PUSH` and `OCCUPY` branches.

So today only `point_0` and `point_1` are actually used unless you edit `strategy_tree.xml`.

This is a deployment-critical detail. If you carefully tuned four occupy points but do not update the tree, two of them will never be visited.

### Step 7. Tune state thresholds after points are fixed

Do not tune decision thresholds before the coordinates are sensible.

The key thresholds in `strategy_decision_tmp.launch` are:

- `danger_hp`
- `max_hp`
- `sufficient_bullet`
- `max_bullet`
- `fixed_supply`
- `occupy_threshold`
- `aggressive_threshold`
- `attack_threshold`
- `harm_threshold_on`
- `harm_threshold_off`

Set points first, then tune state transitions.

Otherwise you end up trying to fix bad spatial behavior with logic thresholds.

### Step 8. Validate DecisionNode alone before combined field launch

Once points are configured, bring up DecisionNode with the rest of the nav stack already healthy and verify:

- it stays in `INIT` before game start
- it transitions to `PUSH` after game start
- it publishes to `/clicked_point`
- arrival on `/dstar_status` causes action progression
- low HP triggers `SUPPLY`
- strong advantage plus sufficient bullets triggers `RADICAL` and `RADICAL1`

This is where you verify the logic, not the base navigation.

## Practical Launch Sequence For Real Deployment

This is the sequence I would recommend if you are already familiar with old_nav.

### Phase A. Environment bring-up

1. Power the robot, lower computer, LiDAR, and MCU.
2. Confirm the expected serial device exists, normally `/dev/ttyUSB0`.
3. Source ROS and all overlaid workspaces in this order:
   - `/opt/ros/noetic`
   - `Old_nav/livox_ws/devel`
   - `Old_nav/sim_nav/devel`
   - `Old_nav/Navigation-filter-test/devel`
   - `DecisionNode/devel`

The provided autostart script already does this.

### Phase B. Navigation stack bring-up

1. Launch the old_nav stack.
2. `map_server` publishes the 2D map.
3. IMU filtering starts.
4. LiDAR driver starts.
5. `hdl_localization` loads the PCD and begins pose estimation.
6. `real_robot_transform` and `ser2msg_decision_givepoint` establish the planning TF chain, including `virtual_frame`.
7. `dstarlite` waits for a goal on `/clicked_point`.

You should not proceed until:

- the pose is stable in RViz
- the robot location is consistent in the map frame
- manual `/clicked_point` goals work

### Phase C. Decision and MCU bring-up

1. Start `mcu_communicator`.
2. Confirm it publishes referee and robot status topics.
3. Start `strategy_node`.
4. Confirm it loads `strategy_tree.xml` and publishes initial default outputs.

Now the autonomy loop is complete:

- DecisionNode sees game and robot state
- DecisionNode publishes `/clicked_point`
- old_nav plans and drives
- arrival feeds back to DecisionNode

### Phase D. Match or mission runtime behavior

A typical runtime sequence is:

1. Launch completes.
2. Strategy node initializes and stays in `INIT` while game is not started.
3. Game starts and DecisionNode moves from `INIT` to `PUSH`.
4. `PUSH` publishes the first `occupy` point to `/clicked_point`.
5. old_nav drives to that point.
6. When `/dstar_status` indicates arrival, the tree can switch to `OCCUPY`.
7. In `OCCUPY`, the robot cycles through configured occupy points.
8. If HP becomes dangerous, the tree changes to `SUPPLY` and publishes the supply point.
9. If tactical advantage is high and bullets are sufficient, the tree may switch to `RADICAL`, then `RADICAL1`.
10. If dead, it transitions to `RESPAWN`; after recovering alive, it moves to `SUPPLY`.
11. If no stronger branch applies, it can fall back to `WAITFOROP`, and under intense harm use the retreat point.

That is the operational state machine you should think about when assigning points.

## What I Would Configure Before Every Real Deployment

For each venue or field, I would verify these items in exactly this order.

### Map consistency

- `base.yaml` points to the correct `.pgm`
- `base.yaml` origin and resolution are correct
- `hdl_localization.launch` points to the correct `.pcd`
- the `.pgm/.yaml` and `.pcd` describe the same field frame

### Localization

- initial pose in `hdl_localization.launch` is valid for the actual start spot
- LiDAR and IMU topics are healthy
- pose is stable in RViz

### Transforms

- `virtual_frame` is present
- `map -> virtual_frame` looks correct
- no unexpected rotation or map offset

### Planner

- `/clicked_point` manual goal works
- `/dstar_status` updates correctly on arrival
- `/cmd_vel` looks sane

### Decision points

- `supply` is safe and reachable
- `waitforop` is a reasonable idle point
- `retreat` is actually safer than `waitforop`
- `radical` and `radical1` are aggressive but still navigable
- `occupy` points represent the intended patrol or contest path
- `point_count` in the behavior tree matches how many occupy points you expect to use

### Runtime topics

- `/referee/game_progress`
- `/referee/remain_hp`
- `/referee/bullet_remain`
- `/referee/occupy_status`
- `/dstar_status`
- `/clicked_point`

If any of these are missing, the autonomy will either remain stuck in a default state or make bad transitions.

## Important Repository-Specific Notes

### 1. The combined launch still includes old_nav's `ser2msg_decision_givepoint`

`3DNavUL_Test.launch` includes `ser2msg_tf_decision_givepoint.launch`.

In the current source, `ser2msg_decision_givepoint.cpp` has its serial port initialization disabled and its `clicked_point` publish path commented out. In practice it is mainly acting as a TF and helper bridge, not as the active decision source.

That is why the combined launch can coexist with DecisionNode's own `mcu_communicator` even though both launch files still carry `serial_port` parameters.

Still, treat this carefully. If someone re-enables serial or re-enables `clicked_point` publishing in the old node, you can create duplicate goal sources or serial contention.

### 2. DecisionNode is not a replacement for map tuning

If D* Lite performs poorly with a point, changing BT thresholds is not the right first move.

Fix one of these first:

- the point itself
- the map
- localization
- TF alignment

### 3. Keep one deployment profile per field

The cleanest deployment pattern is to create one field-specific decision launch or config variant for each map package instead of editing values ad hoc before every run.

For example, keep one profile per venue containing:

- map yaml name
- pcd path
- init pose
- decision point set
- any field-specific threshold overrides

## Recommended Command Path

If you want the existing integrated bring-up path, use:

```bash
cd /home/sentry/AstarTraining/Old_nav
bash run_3DNavUL_Test_with_decision.sh
```

Or use the combined launch directly after sourcing all workspaces:

```bash
roslaunch /home/sentry/AstarTraining/Old_nav/3DNavUL_Test_with_decision.launch
```

For incremental debugging, do not start there first. Validate old_nav alone before using the combined launch.

## Short Deployment Checklist

Before every autonomous run, confirm:

- correct `.yaml` and `.pgm` loaded into `map_server`
- correct `.pcd` loaded into `hdl_localization`
- correct initial pose
- stable localization
- working TF chain
- manual `/clicked_point` navigation works
- DecisionNode points match the actual field
- occupy point count in the XML matches the intended route length
- referee and status topics are alive
- DecisionNode publishes `/clicked_point`

If all of those are true, the actual sequence of events during deployment is simple:

1. bring up old_nav until localization and planning are healthy
2. bring up MCU and DecisionNode
3. wait for game start
4. let DecisionNode publish strategic points
5. let old_nav execute those points and feed arrival back
6. verify state transitions match your tactical intent

That is the correct mental model for deploying this stack.