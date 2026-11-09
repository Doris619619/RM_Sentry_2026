<!-- 文件用途：提供 RM_Sentry_2026 的开发协作规范和基础仿真使用文档入口。 -->

# RM_Sentry_2026

本仓库包含哨兵导航项目及 ROS 2 工作区 `ros2_ws`。

## 开发协作

修改代码、配置、文档或 Git 分支前，阅读 [工程协作规范](docs/工程协作规范.md)。
创建或更新 PR 时，遵守 [PR撰写规范](docs/PR撰写规范.md)。

这两份规范按用户要求从 Threadline 原样引入，保留原文标题和示例，在本仓库采用其中的协作规则。
功能分支使用 `<type>/<YYYYMMDD>-<short-description>`；主分支 `master` 保留。
旧分支改名使用其原始创建日期，不改写提交历史。

## 基础导航仿真

运行环境为 VMware Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress。
[中文使用说明](docs/simulation/README.md) 包含启动、键盘控制和停止命令；
[验收报告](docs/simulation/ACCEPTANCE.md) 包含实际截图、运动反馈和性能数据。
第一阶段提供理想平面手动运动与位置反馈，原入口保持可回退。

第二阶段已接通地图对应场地、真实雷达点云和 RViz 目标/路径展示，运行方法见 [第二阶段说明](docs/simulation/part2/README.md)。
**第二阶段验收通过：15/15 次轨迹检查、加速度上限、双 GUI 性能及退出复验通过。**
已按用户后续授权修复轨迹时长/系数失配与加速度提前结束检查；自动跟踪仍留在第三阶段。
[验收报告](docs/simulation/part2/ACCEPTANCE.md)、[五页队会提纲](docs/simulation/part2/TEAM_REPORT.md) 和 [演示视频](docs/simulation/part2/demo.mp4) 已归档。

## 六项自主导航仿真

六项分别通过：MPC 自动跟踪、安全异常与接管、动态障碍重规划、实际 NDT 定位、
通用底盘接触动力学、真实决策与 MCU 程序的完整模拟串口联调。
15/15 组次自动到点；完整物理往返误差 4.84 / 6.55 cm；
双 GUI 310 秒平均实时率 0.853、RSS 峰值 2.05 GiB。

- 完整演示入口：`bash scripts/gazebo_full.sh`；启动后由模拟裁判场景触发真实决策。
- 真值定位的分层回归入口：`bash scripts/gazebo_autonomy.sh`。
- 独立物理底盘与坡道入口：`bash scripts/gazebo_physics.sh`。
- [启动、停止与完整接口](docs/simulation/autonomy/完整系统接口.md)
- [六项验收报告与失败记录](docs/simulation/autonomy/验收报告.md)
- [队会汇报与真车前交接](docs/simulation/autonomy/队会汇报与交接.md)

底盘是通用全向力模型，MCU 连接专属 PTY；没有连接电控板。
定位使用真实仿真雷达和已知位置附近的先验，不能等同于真车验收、
未知起点全局重定位或比赛场地三维还原。第一、二阶段入口继续保留。

## 可替换定位与地图的独立导航入口

第二步保留原有规划器与 MPC 算法，新增只启动规划、MPC 和速度桥接的入口：

```bash
ros2 launch sentry_bringup navigation.launch.py \
  map_profile:=/absolute/path/to/map-profile \
  odom_topic:=/your_lio/odometry cloud_topic:=/your_lio/world_cloud
```

队友按 `map` 位姿、`base_link` 速度及世界系点云契约接入；地图由一套配对的
OCC/BEV/Topo PNG 和元数据提供。输出默认是 `/navigation/cmd_vel`，接入底盘前须经过速度安全门。
接口、替换步骤和限制见 [导航接口说明](docs/navigation_interfaces.md)。

RMUC2024 地图沿用团队仓库已合入的上游资源，来源见 [场景说明](docs/simulation/HIT原开源赛场场景.md)。
纯导航仿真使用 `bash scripts/gazebo_navigation.sh`，支持 RViz 的 2D Goal Pose 或 `/goal`。
本段涉及的新地图结果单独记录；上面的历史 15/15 数据属于原地图，不能作为新场景成功率。
