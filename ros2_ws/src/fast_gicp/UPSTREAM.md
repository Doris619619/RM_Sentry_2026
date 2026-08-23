<!-- 此文件用于记录 fast_gicp ROS2 基线的固定上游来源与启用范围。 -->

# 上游基线

- 上游仓库：`Linlinqiu/hdl_localization_ros2`
- 固定提交：`34a6ad4a74380dba362ecca4dc2d2d8df06ed617`
- 引入范围：CPU `fast_gicp` 库及其公开头文件。

# 本阶段策略

- `BUILD_VGICP_CUDA=OFF`，不引入 CUDA、GPU 或上游样例点云。
- HDL 默认仍选用 `NDT_OMP`；此包用于固定、可复现地保留 CPU GICP 备选实现。
