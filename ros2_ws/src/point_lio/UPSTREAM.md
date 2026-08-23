<!-- 此文件用于记录本仓库 Point-LIO ROS2 基线的上游来源与本地适配边界。 -->

# Point-LIO ROS2 上游基线

- 上游仓库：`https://github.com/dfloreaa/point_lio_ros2`
- 固定提交：`a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7`
- 引入日期：2026-08-23

本目录基于上述提交导入。项目内仅适配左 MID360 的 `CustomMsg`、参数化 QoS/IMU 缓存、`odom -> base_link` TF 与统一输出话题；算法主体保持上游实现。后续变更必须记录在 Git 提交中，不得静默更新上游基线。

双雷达融合保持关闭，不作为本阶段 Point-LIO 验收条件。
