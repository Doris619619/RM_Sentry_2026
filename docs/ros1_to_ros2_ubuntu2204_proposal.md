# RM_Sentry_2026：ROS 1 至 ROS 2（Ubuntu 22.04）迁移 Proposal

> 状态：提案，尚未修改任何业务节点  
> 审计日期：2026-07-26  
> 审计基线：`master` / `6a165c1d91fb6c5c1bb2526ac99d60f819bf51cd`  
> 目标平台：Ubuntu 22.04.5 LTS + ROS 2 Humble（默认 RMW：Cyclone DDS）

## 1. 结论

本仓库不能通过把 `catkin` 批量替换为 `ament_cmake` 完成迁移。它当前由五个独立 catkin 工作区组成，且同时包含自研决策、感知处理、规划控制、SLAM/定位和仿真代码；其中 OCS2、Point-LIO、HDL 图优化/定位栈均深度使用 ROS 1 API。

建议采用 **新建 ROS 2 colcon 工作区、按数据流分批原生迁移、最后切换启动入口** 的方式，不在现有 ROS 1 工作区上原地升级。首个可运行里程碑应为“Livox → 点云处理 → Point-LIO/定位 → TF/RViz2”；第二个为“全局/局部规划与 OCS2 跟踪”；最后迁移行为树决策、串口 MCU 与仿真工具。过渡期间如必须接 ROS 1 外设，可临时使用 `ros1_bridge`，但不得将其作为最终运行时依赖。

## 2. 审计事实与范围

### 2.1 当前运行环境

- 主机为 Ubuntu 22.04.5 LTS（kernel `6.8.0-124-generic`），但当前会话未设置 `ROS_DISTRO`，且未发现 `colcon` 或 `catkin` 命令。
- 仓库顶层包含：`DecisionNode`、`ws_livox`、`ws_cloud`、`HIT_code/sentry_planning_ws`、`HIT_Integrated_test/sim_nav` 五个带 `.catkin_workspace` 的工作区；每个均已有 `build/` 与 `devel/` 产物。
- 基线存在未提交内容：两份规划 launch 文件被修改，`docs/`、`run_planning_sim.sh`、`send_goal.sh` 未跟踪。迁移须在单独分支执行，且不得覆盖这些文件。
- `git submodule status` 因 `HIT_Integrated_test/sim_nav/src/FAST_LIO` 缺少 `.gitmodules` 映射而失败；这是迁移前必须修复的仓库完整性问题。

### 2.2 组件清单与处置

| 域 | 当前位置 / 包 | 现状 | ROS 2 处置 |
|---|---|---|---|
| 决策与串口 | `DecisionNode/src/decision_node` | `roscpp`、BehaviorTree.CPP v3、`serial`、TF；`strategy_node`、`mcu_communicator` 等 4 个可执行文件 | 重写为 `rclcpp` 节点；保留业务状态机，替换参数、定时器、串口和 QoS；升级/验证 BehaviorTree.CPP 依赖 |
| 雷达驱动 | `ws_livox/src/livox_ros_driver2` | CMake 已按 `ROS_EDITION` 分 ROS1/ROS2，已有 `launch_ROS2/*.py` 与 `ros2_headers.h` | 不重写驱动；切到其 ROS2 构建路径，统一为 ament 包，并以实际 Livox 设备验收 `CustomMsg` / PointCloud2 |
| 点云融合与栅格 | `ws_cloud/src/livox_cloudpoint_processor` | ROS1 `roscpp` + PCL，双雷达订阅，发布 PointCloud2、OccupancyGrid 和过滤结果 | 改为一个 `rclcpp` 节点；参数化输入/输出 topic，明确 SensorDataQoS 与 frame/timestamp 策略 |
| 规划消息 | `sentry_msgs`、`trajectory_generation/msg/trajectoryPoly.msg` | ROS1 `message_generation`；含裁判状态、`GoTarget.srv`、轨迹多项式 | 先迁移到 `rosidl_default_generators`；所有消息改用 `std_msgs/msg/Header`，服务改为 `sentry_msgs/srv/GoTarget` |
| 全局规划 | `trajectory_generation` | `roscpp`、PCL、OpenCV、TF、visualization_msgs；约 40 个 ROS API 源文件（与 tracking/rviz 合计） | 迁移为 `rclcpp`；保留算法类，提取 ROS adapter；将 XML launch 改为 Python launch |
| 跟踪与 MPC | `trajectory_tracking` + vendored `ocs2` | `tracking_node`、`hit_bridge`；依赖 catkin 版 `ocs2_ros_interfaces`、`ocs2_msgs`、`ocs2_mpc` 等 | 高风险项：不要手工逐文件移植当前 vendored OCS2；先以与 Humble 兼容的 OCS2 ROS2 上游版本替换，再把 `ocs2_sentry` 适配到其接口 |
| RViz 插件 | `rviz_plugins` | ROS1 `rviz`、Qt5、pluginlib 工具与 display | 改为 `rviz_common` / `rviz_rendering` / `rviz_default_plugins` 体系，使用 ament plugin export；RViz1 plugin XML 不能直接复用 |
| 建图与定位 | `Point-LIO`、`hdl_localization`、`hdl_global_localization`、`hdl_graph_slam`、`ndt_omp`、`fast_gicp` | ROS1 nodelet、pluginlib、TF、PCL、自定义 msg/srv | 按上游 ROS2 端口替换或 fork 后迁移；nodelet 先拆为独立 `rclcpp` node，稳定后再改 `rclcpp_components` 组合 |
| 仿真/导航 | `bot_sim`、`bot_sim_stable`、D* Lite/DWA 与 launch | ROS1 topic 图，部分接 `/move_base`、TEB 与 costmap_converter | 不直接移植 move_base/TEB 依赖；改接 Nav2（planner/controller/costmap）或保留自研 D* Lite/DWA 为 ROS2 节点并定义兼容 topic |

## 3. 已确认的接口影响

- `strategy_node` 发布 `clicked_point`、`motion`、`recover`、`bullet_up`、`bullet_num`；`mcu_communicator` 发布大量裁判、机器人和雷达 topic，并使用 1 个 ROS1 timer。迁移后应保持消息语义与 topic 名称，除非在接口版本表中明确批准改名。
- 点云处理节点订阅左右扫描，发布原始/过滤 PointCloud2 及两个 OccupancyGrid；其 ROS2 版本必须先固定 frame、时间戳、是否 transient-local 与队列深度，避免 DDS 下丢帧或地图不同步。
- `sentry_msgs` 已定义 `RobotsHP`、`RobotStatus`、`slaver_speed` 和 `GoTarget`；轨迹包定义 `trajectoryPoly`。HDL 另有 ScanMatchingStatus、全局定位服务和图保存/加载服务；这些都必须在其消费者迁移前完成 rosidl 生成。
- `hdl_localization` 与 `hdl_graph_slam` 使用 nodelet 和 pluginlib；ROS2 不存在 nodelet ABI 兼容层，必须迁移为普通节点或 component。
- 当前 OCS2 ROS 接口直接使用 `ros::NodeHandle`、Publisher/Subscriber、Service、CallbackQueue、`ros::master::check()` 和 catkin；它是仓库中最大的 API 迁移风险。

## 4. 目标架构

```text
Livox ROS2 driver
  -> cloud_processor_ros2 -> Point-LIO ROS2 / localization ROS2
  -> TF2 + map/odom/base_link -> planning_generation_ros2
  -> tracking_mpc_ros2 (ROS2 OCS2) -> chassis command bridge
MCU serial bridge -> referee / robot state -> decision_bt_ros2 -> goals / mode commands
RViz2 + ROS2 launch 贯穿上述节点
```

仓库目标布局建议如下（`src/` 只保留源码，禁止提交 `build/`、`install/`、`log/`）：

```text
ros2_ws/
  src/
    rm_sentry_interfaces/
    rm_sentry_livox/
    rm_sentry_perception/
    rm_sentry_localization/
    rm_sentry_planning/
    rm_sentry_control/
    rm_sentry_decision/
    rm_sentry_rviz/
    third_party/ocs2_ros2/        # 固定 commit 或 rosdep 来源
  build/ install/ log/             # .gitignore
```

## 5. 关键技术映射

| ROS1 | ROS2 Humble |
|---|---|
| `catkin` / `catkin_make` / `catkin build` | `ament_cmake` / `colcon build --symlink-install` |
| `roscpp` / `ros::NodeHandle` | `rclcpp` / `rclcpp::Node` |
| `advertise` / `subscribe` | `create_publisher` / `create_subscription`，显式 QoS |
| `ros::Rate`、`ros::spinOnce` | wall timer + `rclcpp::spin` / executor |
| `ros::param`、私有参数 | `declare_parameter` / parameter callback / YAML |
| `dynamic_reconfigure` | ROS2 parameters；当前 cloud 包未见实际 `.cfg`，可移除其未使用依赖 |
| `tf` | `tf2_ros` + `tf2_geometry_msgs` |
| `nodelet` | 独立 node，或后续 `rclcpp_components` |
| `message_generation` / `message_runtime` | `rosidl_default_generators` / `rosidl_default_runtime` |
| ROS1 XML `.launch` | Python `launch` / `launch_ros.actions.Node` |
| `rosbag` | `rosbag2`；使用 MCAP 或 sqlite3 并验证回放时钟 |

## 6. 分阶段实施计划

### Phase 0：冻结基线与环境（完成条件：可重复构建空 ROS2 overlay）

1. 修复 FAST_LIO submodule 映射，记录全部 third-party 的来源、commit、许可证和 ROS2 状态。
2. 新建 `ros2-migration` 分支；保留现有 ROS1 工作区不动，清理策略仅作用于新 `ros2_ws` 的生成目录。
3. 安装 ROS2 Humble、`colcon`、`vcstool`、rosdep、PCL/OpenCV/Eigen/Boost、serial、Cyclone DDS；执行 `rosdep install --from-paths src --ignore-src -r -y`。
4. 生成接口契约：topic、type、frame、频率、QoS、参数默认值、服务语义与 launch 参数；以 ROS1 录包作为对比金标准。

### Phase 1：接口、驱动和感知（完成条件：真实雷达到稳定 PointCloud2/OccupancyGrid）

1. 创建 `rm_sentry_interfaces`，迁移 `sentry_msgs`、`trajectoryPoly` 及仍需保留的 HDL 消息/服务。
2. 构建 Livox driver 的 ROS2 路径；不同时加载其 ROS1 package.xml/catkin 目标。
3. 迁移 cloudpoint processor，先保持算法与 topic 语义，增加参数 YAML、`sensor_data` QoS、诊断和单元测试。
4. 以 rosbag2 回放和实机各验证一次：点数、frame_id、时间单调性、栅格尺寸/分辨率、CPU 和端到端延迟。

### Phase 2：定位、TF 与可视化（完成条件：`map -> odom -> base_link` 连续可用）

1. 为 Point-LIO、fast_gicp、ndt_omp、HDL 各自决定“采用维护中的 ROS2 上游端口”或“本仓库 fork 迁移”；禁止混用不同 ABI 的 PCL/Sophus。
2. 先将 nodelet 拆成单进程普通节点，确认语义后再进行 component 化和进程内零拷贝优化。
3. 将 RViz 插件迁移到 RViz2；同时提供无自定义插件的基础 RViz2 配置，避免插件阻塞核心链路验收。

### Phase 3：规划与控制（完成条件：ROS2 端到端规划、跟踪和急停）

1. 迁移 trajectory generation、waypoint generator、tracking 的 ROS adapter；算法核心尽量保持纯 C++、可脱离 ROS 测试。
2. 以 ROS2 兼容 OCS2 替换当前 catkin 版 OCS2，再适配 `ocs2_sentry` dynamics/cost/constraint；逐个验证 observation、target trajectory、policy 和 reset service。
3. 将 `move_base`/TEB 依赖明确替换为 Nav2，或为自研 D* Lite/DWA 定义 ROS2 controller 接口；不要把 ROS1 导航栈桥接进最终部署。
4. 定义控制 topic 的可靠性、deadline、liveliness 与失联安全策略，验证急停/串口断连时底盘零速度。

### Phase 4：决策、MCU 与系统交付（完成条件：实机全链路运行）

1. 迁移 BehaviorTree 决策与 MCU 串口桥；对裁判状态、导航状态、目标点和底盘指令建立消息契约测试。
2. 把所有 ROS1 launch 合并为分域 ROS2 Python launch，提供 `sim`、`hardware`、`debug` 三套入口。
3. 提供 systemd/容器化部署说明、rosbag2 诊断记录、参数文件、RMW 配置和回滚到 ROS1 的操作。

## 7. 验收门槛

- `colcon build --symlink-install` 在干净 Ubuntu 22.04 + Humble 环境通过，`rosdep` 无未声明依赖。
- 每个接口包能独立生成 C++/Python 类型支持；服务和消息字段与 ROS1 基线逐项一致。
- 感知、定位、规划、控制、决策均有 launch test 或 rosbag2 回放测试；关键节点有启动、参数、TF、topic、服务和退出码检查。
- 实机连续运行至少 30 分钟：无 TF 时间回跳、无关键 topic 丢失、无内存持续增长；对 ROS1 基线比较延迟、点云频率、定位漂移和规划成功率。
- 安全验收：DDS QoS 不兼容有明确日志；串口/雷达/定位失联会触发安全降级；任何控制命令默认不会在无有效心跳时持续输出。

## 8. 风险与决策点

1. **OCS2 与定位栈是主路径风险**：当前 vendored 代码均为 ROS1/catkin，建议优先确认可用的 ROS2 上游 commit 与许可证，不把其迁移工作隐藏在应用包改造中。
2. **API 兼容不等于时序兼容**：ROS2 DDS 的 QoS、发现、队列和时钟行为会改变点云/控制链路；必须以接口契约和录包回归验证。
3. **不要直接迁移生成目录**：现有 `build/`、`devel/` 是 ROS1 产物，ROS2 仅使用 `build/ install/ log/`，并加入忽略规则。
4. **系统级切换必须可回滚**：在 Phase 3 以前，实机控制保持 ROS1 基线或使用隔离测试平台；`ros1_bridge` 仅作短期联调。

## 9. 首个实施 PR 的建议边界

首个 PR 只做 Phase 0 + `rm_sentry_interfaces` + Livox ROS2 构建验证：建立 `ros2_ws`、接口包、依赖清单、`.gitignore`、最小 Python launch、CI build 和接口测试。不要在同一个 PR 中迁移 OCS2、Point-LIO、HDL 或决策业务逻辑。

这样可先验证 Ubuntu 22.04/Humble 工具链、消息兼容性与 Livox 实机数据，再用可测量的接口契约分批迁移高风险模块。