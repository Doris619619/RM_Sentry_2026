# ROS1 → ROS2 Planning 等价迁移对照

算法基线：`HIT_code/sentry_planning_ws/src/sentry_planning/trajectory_generation` 与
`waypoint_generator`。本包的正式运行目标是 `trajectory_generator_node`；它不链接或调用
已删除的 `PlannerCore`、简化 A*、shortcut 或自然三次样条。

| ROS1 文件/职责 | ROS2 对应实现 | 迁移方式 |
| --- | --- | --- |
| `RM_GridMap.{h,cpp}`、`node.h` | `port/include/trajectory_generation/RM_GridMap.h`、`port/src/RM_GridMap.cpp` | 保留静态/双高度/拓扑/局部 PCL 地图、膨胀和栅格坐标算法；ROS 发布层由兼容边界替换。 |
| `Astar_searcher.{h,cpp}`、`backward.hpp` | 同名 `port/` 文件 | 保留碰撞、可见性、邻点、裁剪和地图可视化算法。 |
| `TopoSearch.{h,cpp}` | 同名 `port/` 文件 | 保留守卫点、图连边、Dijkstra、拓扑等价与局部图；`planner.test_random_seed=-1` 默认仍用原随机设备，非负值仅供对照测试。 |
| `path_smooth.{h,cpp}`、`root_solver/*` | 同名 `port/` 文件 | 保留 MINCO/L-BFGS、重采样和梯形时间。 |
| `reference_path.{h,cpp}` | 同名 `port/` 文件 | 保留参考速度、姿态、时间和三次多项式矩阵。 |
| `plan_manager.{h,cpp}` | 同名 `port/` 文件 | 保留全局/局部选择、原路径重锚定和重规划。 |
| `replan_fsm.{h,cpp}`、`trajectory_generator_node.cpp` | `port/src/replan_fsm_node.cpp` | 保留 `INIT → WAIT_TARGET → GEN_NEW_TRAJ → EXEC_TRAJ → REPLAN_TRAJ` 和 1 秒冷却；替换为 rclcpp、tf2 和 ROS2 参数。决策/裁判/Gazebo/轮速接口仍隔离，未默认订阅。 |
| `visualization_utils.{h,cpp}` | 同名 `port/` 文件 + ROS2 Marker publisher 绑定 | 全部原规划阶段 Marker 均有 ROS2 topic。 |
| `waypoint_generator.cpp`、`sample_waypoints.h` | `waypoint_generator/src/waypoint_generator_node.cpp` | 保留手动、noyaw、series、free/point/circle/eight、触发与 PoseArray 语义。 |

## ROS2 API 边界

- `roscpp`、NodeHandle、Time、日志和 catkin 分别替换为 `rclcpp`、声明参数、`rclcpp::Time`、RCLCPP 日志和 ament/colcon。
- 动态点云使用消息时间完成 `map <- cloud_frame` tf2 变换。无 TF 时该帧被丢弃并限频告警，不写入全局地图。
- 地图通过 ament index 查找；启动时缺图、尺寸不符或核心参数非法会失败退出。
- 输出仅为 `/global_trajectory`，每段 4 个 X、4 个 Y 系数和正 duration；`start_time` 是实际 ROS2 发布时间。

## 启动入口

- `global_planning.launch.py`：正常运行。
- `global_planning_sim.launch.py`：软件/仿真入口，无 Gazebo 依赖。
- `global_planning_debug.launch.py`：ROS1 debug 对应入口，默认 `gdb -ex run --args`；可用 `planner_prefix:=` 覆盖。

## 可复现实验

`test/test_legacy_planning.cpp` 验证坐标/栅格往返、边界钳制、越界占据、静态占据目标邻点修正、动态点云高度阈值占据以及 Topo → 平滑 → 参考多项式链。
`test/mock_replan_fsm.py` 验证无里程计不发布、静态规划、缺 TF 丢帧、有效 TF 动态点云、冷却和轨迹数学约束。
`waypoint_generator/test/mock_waypoint_generator.py` 验证 manual、noyaw、point、free、circle、eight 和 series。
ROS1 与 ROS2 对照使用相同地图、起终点 `(-3.82, 2.40) → (-1.35, -4.20)` 和种子 `7`；产物保存为 JSON，比较末端误差、采样路径长度和总时间。

| 指标 | ROS1 Noetic | ROS2 Humble | 结果 |
| --- | ---: | ---: | --- |
| 多项式段数 | 15 | 15 | 一致 |
| 终点 `(x, y)` | `(-1.369000, -4.204000)` | `(-1.369000, -4.204000)` | 误差 0 m |
| 采样路径长度 | 8.484419 m | 8.484419 m | 相对误差 0% |
| 总时长 | 4.885743 s | 4.885743 s | 相对误差 0% |

## 待实机验证

Livox/双雷达外参、定位精度、实际动态障碍、底盘/OCS2 跟踪、决策与裁判接口、比赛地图均未在本阶段伪装为已通过。
