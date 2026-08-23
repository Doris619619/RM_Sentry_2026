<!-- 此文件用于记录 HDL ROS2 定位基线的上游来源与本仓库适配边界。 -->

# 上游基线

- 上游仓库：`Linlinqiu/hdl_localization_ros2`
- 固定提交：`34a6ad4a74380dba362ecca4dc2d2d8df06ed617`
- 引入范围：`hdl_localization` 的 CPU NDT_OMP 定位基线、消息和地图服务器。

# 本仓库适配

- 默认输入为 `/filted_topic_3d` 和 `/point_lio/odometry`。
- 默认关闭 HDL 直接 IMU 融合及全局重定位。
- 只发布 `map -> odom`，计算式为 `T_map_base * inverse(T_odom_base)`；禁止回退发布 `map -> base_link`。
- 外部 PCD 必须经启动参数 `globalmap_pcd` 提供，不提交地图数据。
