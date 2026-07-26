# ROS 2 Humble 工作区

此目录仅承载 ROS 2 迁移后的源码与构建产物，与仓库内既有的 ROS 1 catkin 工作区完全隔离。不要在 `ws_livox/` 中切换 `package.xml` 或运行 ROS 2 构建脚本。

## 已固定的外部依赖

`livox_ros_driver2.repos` 固定检出官方 `Livox-SDK/livox_ros_driver2` 的 `1.0.0` release commit `dd6c8de14479197e314270af8133f8a4cbe16ff9`。该版本与本仓库现有 Livox 驱动版本一致，并支持 Ubuntu 22.04 上的 ROS 2 Humble；第一阶段不升级驱动版本，以保持 `CustomMsg` 接口和下游 Point-LIO 预期稳定。

## 前置条件

- Ubuntu 22.04 + ROS 2 Humble，且可执行 `source /opt/ros/humble/setup.bash`。
- 已安装 `python3-vcstool`、`python3-colcon-common-extensions`、PCL 与驱动的 ROS 2 依赖。
- 仓库根目录的 `Livox-SDK2` 已编译并安装 shared library。按其 README 在 `Livox-SDK2/build` 中执行 `cmake .. && make -j`、`sudo make install`，然后执行 `sudo ldconfig`。驱动构建需要 `/usr/local/lib/liblivox_lidar_sdk_shared.so` 与对应头文件。

## 导入与构建

```bash
cd /home/liangys/RM_Sentry_2026/ros2_ws
mkdir -p src
vcs import src < livox_ros_driver2.repos

source /opt/ros/humble/setup.bash
cd src/livox_ros_driver2
./build.sh humble

source ../../install/setup.bash
ros2 pkg executables livox_ros_driver2
```

`build.sh humble` 只允许从 `ros2_ws/src/livox_ros_driver2` 执行：该脚本会清理其上两级目录的 `build/`、`install/` 和 `devel/`，在此工作区中这是预期行为；在现有 ROS 1 `ws_livox` 中执行会污染其构建状态。

## 最小验证

构建完成后，先确认 `livox_ros_driver2_node` 出现在 `ros2 pkg executables livox_ros_driver2` 的输出中。连接真实设备并使用对应 JSON 配置后，再通过 `ros2 launch livox_ros_driver2 msg_MID360_launch.py` 验证 ROS 2 的 `CustomMsg`、帧名和时间戳；下游点云处理、Point-LIO 与仿真节点不属于本提交范围。
