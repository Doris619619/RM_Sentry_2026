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
当前阶段提供理想平面手动运动与位置反馈，后续地图、点云和导航接入见使用说明中的第二阶段交接。

第二阶段的场地、雷达点云和规划展示进度见 [第二阶段说明](docs/simulation/part2/README.md)。
