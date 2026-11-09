# ROS 2 仿真启动指南

本文说明如何在 Ubuntu 桌面环境中构建和运行本仓库的 ROS 2 Gazebo 仿真。项目同时保存有 ROS 1/Noetic 工程；本文只针对 `ros2_ws/` 和 `scripts/gazebo_*.sh`，不要用根目录的 `run_planning_sim.sh` 或 `send_goal.sh` 启动 ROS 2 仿真。

## 日常使用：一键启动与菜单控制

本机已完成构建，可以直接使用仓库根目录的 `sim.sh`。脚本根据自身位置查找项目，自动加载 ROS 2、依赖和仿真包，无需手动 `cd`、`source` 或发布消息。

默认场景使用 HIT 开源仓库配套的 **RMUC2024 场地**：Gazebo 加载原赛场网格，规划器同时读取原始 `occ2024low`、`bev2024low` 和 `occtopo2024low`；RViz 青色线直接显示上游完整车道骨架，橙色点是实时仿真雷达扫描。地图坐标原点为 `(0, 0)`、分辨率为 `0.1 m`。上游文件版本和 ROS2 场景适配说明见[场景说明](HIT原开源赛场场景.md)。

在 Ubuntu 桌面终端，从任意目录启动：

```bash
/home/nvidia3/Codes/HKU_Astar_Nav_26/sim.sh
```

另开终端打开控制菜单，按数字选择巡逻、返补给、停止、手动接管、查看状态或完整退出：

```bash
/home/nvidia3/Codes/HKU_Astar_Nav_26/sim.sh menu
```

也可以直接调用对应命令：

| 命令 | 操作 |
| --- | --- |
| `sim.sh start` 或不带参数 | 启动仿真，初始静止；在启动终端按 Ctrl+C 完整退出 |
| `sim.sh patrol` | 结束旧场景、切换自动模式并开始巡逻 |
| `sim.sh supply` | 结束旧场景、切换自动模式并触发低血量返补给 |
| `sim.sh stop` | 停止运动，同时结束模拟比赛；Gazebo 保持运行 |
| `sim.sh keyboard` | 手动接管，按键盘终端提示操作 |
| `sim.sh status` | 显示 ROS 节点和 MCU 状态 |
| `sim.sh restart` | 完整重启并恢复初始静止状态 |
| `sim.sh quit` | 完整退出已登记的本项目仿真 |
| `sim.sh build` | 修改仿真代码后重新构建 |

表中 `sim.sh` 代表脚本的绝对路径；在仓库根目录也可写成 `./sim.sh`。移动仓库后，使用新的脚本路径即可。

RViz 中 `Planner occupancy` 是静态二维底图，`Simulated lidar (live)` 是 `/aligned_points` 的实时仿真雷达点云，显示为橙色点。默认使用斜视角观察点云高度；可暂时取消勾选 `Planner occupancy`，单独观察实时扫描。

依赖环境按 `install_sim_deps/`、`install_closure_fix/`、`install/` 的顺序查找。默认 ROS domain 为 26；若设置了 `ROS_DOMAIN_ID` 或 `SENTRY_DEPENDENCY_SETUP`，脚本会使用指定值。启动终端与菜单终端必须使用相同 domain。下文保留首次构建及手动操作步骤供排查使用。

## 1. 选择仿真入口

| 入口 | 运行内容 | 适合用途 |
| --- | --- | --- |
| `scripts/gazebo_basic.sh` | Gazebo 基础场景、理想平面底盘、真值里程计和手动控制 | 先确认 Gazebo、窗口和键盘控制正常 |
| `scripts/gazebo_autonomy.sh` | 自主规划和 MPC 跟踪；默认使用真值定位和理想平面底盘 | 分层调试规划与跟踪 |
| `scripts/gazebo_physics.sh` | 通用接触动力学底盘 | 单独观察底盘物理和碰撞 |
| `scripts/gazebo_full.sh` | 完整决策、NDT 定位、规划、MPC、模拟 MCU 串口和 Gazebo 力驱动底盘 | **推荐：运行完整自主导航仿真** |

下文以完整仿真 `gazebo_full.sh` 为主，最后附上基础场景的启动方法。

## 2. 环境要求

- Ubuntu 22.04，ROS 2 Humble，Gazebo Fortress（Humble 对应的 `ros_gz` 环境）。
- 已登录图形桌面的终端。启动脚本检查 `DISPLAY`；纯 SSH 会话通常没有 Gazebo/RViz 可用的显示环境。
- 仓库工作区依赖已准备好，且有可用的 ROS 2 构建工具（`colcon`、`rosdep` 等）。
- 仿真脚本默认使用 `ROS_DOMAIN_ID=26`、`ROS_LOCALHOST_ONLY=1`，并为 Gazebo transport 设置独立分区。

以下命令均从仓库根目录执行。先确认位置：

```bash
cd /path/to/HKU_Astar_Nav_26
```

如果当前终端已经位于项目根目录，可跳过这一步。不要在同一个终端先 source ROS 1/Noetic，再运行这些命令。

## 3. 第一次准备工作区

如果这是已构建过仿真的机器，且 `ros2_ws/install_sim_deps/local_setup.bash` 或 `ros2_ws/install_closure_fix/local_setup.bash` 已存在，可直接跳到第 4 节。它们是 OCS2/HPIPM 等依赖的安装环境，不一定会随 Git 仓库提供。

在新环境中，先按工作区说明导入锁定依赖并构建 ROS 2 工作区：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
./tools/bootstrap_sources.sh
export CMAKE_PREFIX_PATH="$PWD/.deps/livox-sdk2:${CMAKE_PREFIX_PATH:-}"
colcon build --symlink-install
source install/setup.bash
cd ..
```

`bootstrap_sources.sh` 会准备锁定的源码并运行依赖检查；网络或系统依赖尚未满足时，按其报错和 [ROS 2 工作区 README](../../ros2_ws/README.md) 处理。若只准备完整仿真所需的依赖，可在 `ros2_ws/` 目录构建仿真依赖闭包：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
export CMAKE_PREFIX_PATH="$PWD/.deps/livox-sdk2:${CMAKE_PREFIX_PATH:-}"
colcon build --build-base build_sim_deps --install-base install_sim_deps \
  --parallel-workers 2 --packages-up-to trajectory_tracking hdl_localization \
  --cmake-args -DBUILD_TESTING=OFF \
  -DOpenCV_DIR="/usr/lib/$(gcc -dumpmachine)/cmake/opencv4"
cd ..
```

这会从源码构建 OCS2/HPIPM、`sentry_msgs`、规划器、跟踪和 HDL/NDT 定位依赖。`sim.sh build` 也会执行这一步，再构建完整仿真包，避免只编译应用包而遗漏定位运行依赖。完整仿真的专用包会构建到独立的 `install_sentry_autonomy/` 下。

如果按上面的仿真依赖闭包构建，告诉仿真脚本使用 `install_sim_deps/`：

```bash
export SENTRY_DEPENDENCY_SETUP="$PWD/ros2_ws/install_sim_deps/local_setup.bash"
```

若本机已有 `install_closure_fix/`，可不设置 `SENTRY_DEPENDENCY_SETUP`，脚本会默认查找它。也可以显式指定：

```bash
export SENTRY_DEPENDENCY_SETUP="$PWD/ros2_ws/install_closure_fix/local_setup.bash"
```

设置该变量的终端中必须能够找到对应的 `local_setup.bash` 文件。若构建和启动在不同终端进行，每个终端都要设置此变量，或者使用脚本默认目录。

## 4. 构建并启动完整仿真

### 终端 A：构建并运行 Gazebo

在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=1

# 本机依赖若构建在 install_sim_deps/，需要设置这一项；
# 若存在默认的 install_closure_fix/，可省略。
export SENTRY_DEPENDENCY_SETUP="$PWD/ros2_ws/install_sim_deps/local_setup.bash"

bash scripts/gazebo_full.sh build
bash scripts/gazebo_full.sh
```

`build` 会编译完整仿真所需的决策、Gazebo、规划、航点生成和轨迹跟踪包。首次构建需要一些时间。之后没有修改相关代码时，可以跳过 `build`，直接执行启动命令。

脚本启动 Gazebo 和系统节点后，终端会持续输出日志，不要关闭该终端。仿真刚启动时默认为比赛停止状态，机器人应保持静止。

### 终端 B：启动巡逻

打开另一个**新的终端**，在仓库根目录执行：

```bash
cd /path/to/HKU_Astar_Nav_26
source /opt/ros/humble/setup.bash

# 使用本指南构建的仿真依赖环境：
source ros2_ws/install_sim_deps/local_setup.bash

source ros2_ws/install_sentry_autonomy/local_setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=1
export SENTRY_DEPENDENCY_SETUP="$PWD/ros2_ws/install_sim_deps/local_setup.bash"

# 先选择自动模式，再触发模拟裁判的巡逻场景。
bash scripts/gazebo_full.sh auto
ros2 topic pub --once /sim/referee/scenario std_msgs/msg/String '{data: patrol}'
```

命令中的 `auto` 是控制模式，`patrol` 是模拟裁判输入；两步都执行后，策略节点才会发出巡逻目标。仅启动仿真不会让机器人自动移动。

要触发模拟低血量返补给场景，可在同一环境终端发送：

```bash
ros2 topic pub --once /sim/referee/scenario std_msgs/msg/String '{data: supply}'
```

结束比赛/停止策略运动：

```bash
ros2 topic pub --once /sim/referee/scenario std_msgs/msg/String '{data: stop}'
```

## 5. 停止、接管和重启

- **立即切换为停止模式：** 在已完成 ROS 环境 source 的终端运行 `bash scripts/gazebo_full.sh stop`。
- **让模拟裁判结束比赛：** 发布 `/sim/referee/scenario` 的 `stop` 场景，见上节。
- **手动接管：** 运行 `bash scripts/gazebo_full.sh keyboard`，然后按终端提示操作。退出键盘控制不会关闭 Gazebo。
- **完整退出：** 回到终端 A 按 `Ctrl+C`。该 launch 退出时会清理它启动的仿真节点。
- **完整重启：** 在已加载 ROS 2 和依赖环境的终端运行 `bash scripts/gazebo_full.sh restart`，或结束启动终端后重新启动。

从手动接管或故障停车状态恢复巡逻时，先发送裁判 `stop` 场景，再重新运行 `auto` 和 `patrol`，使策略收到新的比赛/目标授权。

完整接口和更多运行约束见[完整系统接口说明](autonomy/完整系统接口.md)。

## 6. 检查仿真是否启动

在终端 B（或另一个已 source 相同 ROS 环境的终端）运行：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /clock
```

`/clock` 应持续更新。检查常用运行状态：

```bash
ros2 topic echo /sim/mcu/status
ros2 topic echo /localization/odometry
```

完整仿真采用 NDT 定位和软件 PTY 模拟串口；`/sim/mcu/status` 中可查看 PTY 和 MCU 状态。这里的模拟串口和底盘不是真实电控板或真车硬件。

## 7. 只启动基础场景

若只想确认 Gazebo、底盘显示和键盘控制，不需要构建完整自主导航链：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=1
bash scripts/gazebo_basic.sh build
bash scripts/gazebo_basic.sh
```

另开终端手动控制：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install_sentry_gazebo/local_setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=1
bash scripts/gazebo_basic.sh keyboard
```

基础场景与完整自主仿真共用启动锁和 ROS domain；不要同时运行两套仿真。基础场景的键盘操作、话题和模型限制见[分层仿真说明](README.md)。

## 8. 常见问题

### 提示找不到 ROS 2 或 `ros2` 命令

在当前终端运行：

```bash
source /opt/ros/humble/setup.bash
```

并确认系统安装的是 ROS 2 Humble。不要在同一个终端混用 ROS 1 和 ROS 2 的 setup 文件。

### 提示缺少 OCS2 依赖环境

`gazebo_full.sh` 默认查找 `ros2_ws/install_closure_fix/local_setup.bash`。若按本指南构建到 `install_sim_deps/`，需在**构建和运行终端**设置：

```bash
export SENTRY_DEPENDENCY_SETUP="$PWD/ros2_ws/install_sim_deps/local_setup.bash"
```

确认该文件已生成，再重试构建。

### 提示没有 `DISPLAY`

在 Ubuntu 已登录的桌面会话中打开终端再运行。远程 SSH 需要正确连接图形显示会话；仅设置一个随意的 `DISPLAY=:0` 不会自动提供 X 权限或桌面会话。

### 提示已有仿真运行 / 启动锁被占用

脚本用同一用户和 ROS domain 的锁阻止重复运行。检查其他终端是否仍有 `gazebo_*.sh` 启动的仿真；优先回到原启动终端按 `Ctrl+C`，不要并行启动第二套仿真。

### Gazebo 窗口黑屏或渲染异常

项目脚本已针对虚拟机图形环境设置软件渲染。保存启动日志并检查图形环境；基础场景的诊断说明见[仿真操作说明](README.md)。不要通过改全局桌面设置来绕过问题。

### 看不到机器人移动

完整仿真启动后默认是停止状态。确认已在终端 B 加载与终端 A 相同的 ROS domain，依次执行了 `auto` 和 `patrol`，并检查 `/sim/mcu/status`、`/localization/odometry` 和启动日志。

## 9. 仿真范围

完整仿真用于联调决策、规划、MPC、NDT 定位、模拟 MCU 字节链路和 Gazebo 底盘模型。它不连接真实 MCU、雷达、电机驱动器或比赛裁判系统；仿真通过也不等于真车硬件验收。更多边界与验收记录见[自主导航仿真说明](autonomy/README.md)和[验收报告](autonomy/验收报告.md)。
