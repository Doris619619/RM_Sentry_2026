# 第一阶段：Gazebo 基础导航仿真

这是 **/home/liangys/RM_Sentry_2026** 的仿真基础，不是同学 rm_sentry_nav 仓库的复现。
运行环境：Windows → VMware Ubuntu64-oc → Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress，原生运行。

## 从 Ubuntu 桌面开始

1. Windows 中打开 VMware，进入并解锁 Ubuntu64-oc。
2. 在 Ubuntu 桌面按 Ctrl+Alt+T 打开终端，运行：

```bash
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_basic.sh
```

首次没有专用安装目录时自动只构建 sentry_gazebo 和 sentry_bringup；以后直接启动。
窗口显示 16×16 米平地、1 米网格、单辆机器人、三个远处的静态展示障碍。红色地面箭头为 +X，绿色为 +Y；机器人红色前脸和金色短杆指向机体 +X。

3. 另开一个 Ubuntu 终端，运行键盘控制：

```bash
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_basic.sh keyboard
```

保持键盘终端获得焦点：

| 键 | 动作 |
|---|---|
| W / S | 前进 / 后退，0.2 m/s |
| A / D | 左横移 / 右横移，0.2 m/s |
| Q / E | 逆时针 / 顺时针旋转，0.3 rad/s |
| 空格 | 停止 |
| Esc | 发出零指令并退出键盘控制 |

按住方向键利用终端按键重复持续运动；松键约 0.3 秒归零。终端无法直接读取松键事件，首次重复前可能有短暂停顿。切换方向替换上一指令，不支持组合键斜向控制；程序接口支持 x/y 同时输入。停止键盘工具不会关闭 Gazebo。

鼠标被 VMware 捕获时，按 **Ctrl+Alt** 释放鼠标返回 Windows。

4. 在启动仿真的第一个终端按 **Ctrl+C** 停止整个仿真。不要只暂停窗口后以为程序已经结束。

重新构建（修改代码后）：

```bash
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_basic.sh build
```

构建输出仅在 ros2_ws/build_sentry_gazebo、install_sentry_gazebo、log_sentry_gazebo，编译并发最多 2。不删除原工作区任何已有输出。
启动脚本锁定同一用户/domain，阻止重复启动，并检查现有定位、时钟、TF 发布者。

## 图形操作与诊断

Gazebo 三维窗口中滚轮缩放；中键拖动环绕、左键拖动平移（也可 Shift+左键拖动环绕）。左上角播放/暂停，统计区显示仿真时间与实时率。
使用 XWayland（QT_QPA_PLATFORM=xcb），设置仅限启动进程，不修改 Ubuntu 全局桌面配置。
从 SSH 直接启动时，DISPLAY、XAUTHORITY、XDG_RUNTIME_DIR 必须来自已登录桌面会话；新手请使用上面的桌面终端命令。

本虚拟机已实测默认 SVGA3D + Ogre2 白屏，因此启动脚本默认启用 CPU 软件渲染（llvmpipe），无需 NVIDIA 独显。
在其他机器排查时，先保存终端错误并运行 `glxinfo -B` 检查实际渲染器。如需明确指定软件模式：

```bash
SENTRY_SOFTWARE_RENDERING=1 bash /home/liangys/RM_Sentry_2026/scripts/gazebo_basic.sh
```

只有更换/修复图形环境后才用 SENTRY_SOFTWARE_RENDERING=0 重试默认图形驱动。
软件渲染性能必须重新测量；“虚拟机开启三维加速”本身不构成图形通过证据。
本次实际渲染器和性能请看 [验收报告](ACCEPTANCE.md)。

## 接口约定

键盘终端出现“控制接口已连接”后再开始操作。

默认 **ROS_DOMAIN_ID=26、ROS_LOCALHOST_ONLY=1**，隔离已有硬件/生产 ROS 图。键盘和启动脚本使用同一设置。
Gazebo transport 分区为 `sentry_basic_用户名_domain`。不要并行启动 fixture_inputs、HDL 定位或生产入口。

需要从第三个终端检查话题：

```bash
source /opt/ros/humble/setup.bash
source /home/liangys/RM_Sentry_2026/ros2_ws/install_sentry_gazebo/local_setup.bash
export ROS_DOMAIN_ID=26
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /localization/odometry
```

| 话题 | 类型 | 内容/坐标系 |
|---|---|---|
| /cmd_vel | geometry_msgs/msg/Twist | linear.x/y 为机体系；angular.z 逆时针为正 |
| /sim/guarded_cmd_vel | geometry_msgs/msg/Twist | 内部保护后的命令，勿绕过保护直接发送 |
| /sim/ground_truth/odometry | nav_msgs/msg/Odometry | Fortress OdometryPublisher 实际模型状态，odom / base_link |
| /localization/odometry | nav_msgs/msg/Odometry | pose 在 map，child_frame_id=base_link，twist 在 base_link |
| /clock | rosgraph_msgs/msg/Clock | Gazebo 仿真时间 |
| /tf、/tf_static | tf2_msgs/msg/TFMessage | map → odom（静态单位变换）→ base_link（实际状态） |

world、map、odom 在此阶段重合，原点/初始航向均为零。机器人视觉几何位于地面上方，base_link 在导航平面 z=0。
物理步长 0.01 秒（100 Hz，适合本阶段无碰撞的理想模型）；反馈目标频率 50 Hz（按仿真时间）；暂停时不推进消息时间。桥接、适配和键盘节点 use_sim_time=true。
Gazebo VelocityControl 接收机体系速度；OdometryPublisher dimensions=2 已输出机体系 twist，因此不再旋转第二次。
定位适配器保留原始消息时间、位姿和速度，仅转换父坐标系标签并发布 TF，**从不积分 /cmd_vel 生成位置**。
理想无噪声反馈 covariance 为零，不代表真车定位精度或真实传感器置信度。

默认保护参数：

- 平面速度向量模长最多 0.5 m/s；转向最多 0.5 rad/s。
- 0.5 秒没有新命令即清零，使用单调真实时间计时。
- 仿真时钟超过 0.2 秒不推进，清除命令并拒收运动；恢复需新指令。
- 模型附加 ResumeGuard 插件，在恢复前两步强制零速度，消除 Fortress VelocityControl 内部暂停缓存造成的首步漂移；不修改位姿或积分生成定位。
- 非法数值命令清零；非法反馈拒绝发布。
- 键盘正常退出重复发送零速度；Ctrl+C 和终端断开也会停止并退出；同一用户/domain 只能启动一个键盘程序。程序异常丢失输入由适配器超时停止。
- 适配器或桥接退出时，launch 关闭整个仿真。

## 模型范围

名称：**基础导航仿真模型**。理想平面全向速度控制，无轮胎、悬架、坡道动力学、传感器噪声或真实驱动器。
没有雷达、IMU、点云、定位算法、规划器、MPC 或自动导航。此阶段不启动生产硬件节点。
模型不提供碰撞几何，静态障碍用于场景展示，不能用于验证碰撞/避障。不是与真车完全一致的整车物理模型。
世界重力为零、仅使用平面输入，因此固定展示高度。禁止通过 GUI 施力/修改姿态并期待此阶段自动复位；重新启动可恢复出生状态。
同学仓库仅作为背景参考，未复制其 Docker/NVIDIA/Jazzy 配置或大型场地。

## 安装、文件和证据

本次新增依赖：`ros-humble-ros-gz`、`mesa-utils` 及 apt 所列依赖；额外的 `xdotool` 仅用于实际鼠标视角验收，正常运行不需要。精确版本、包差异、磁盘数据见 evidence。
在同版本新环境安装：

```bash
sudo apt-get update
sudo apt-get install ros-humble-ros-gz mesa-utils
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_basic.sh build
```

不要重装已有 ROS，也不需要 Docker。sudo 在 Ubuntu 自行输入密码；仓库和脚本不保存密码。

- ros2_ws/src/sentry_gazebo：模型、场地、桥接、保护、键盘、验证工具。
- ros2_ws/src/sentry_bringup/launch/gazebo_basic.launch.py：独立启动入口。
- docs/simulation/evidence：截图、原始运动数据、稳定性、构建/启动日志。
- [验收报告](ACCEPTANCE.md)：实际通过项及尚未解决的问题。

重跑运动验收前应重新启动场景，停止键盘控制；会自动驱动机器人：

```bash
source /opt/ros/humble/setup.bash
source /home/liangys/RM_Sentry_2026/ros2_ws/install_sentry_gazebo/local_setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=sentry_basic_liangys_26
python3 /home/liangys/RM_Sentry_2026/ros2_ws/src/sentry_gazebo/scripts/acceptance.py
```

## 第二阶段交接

先核对现有规划地图的文件来源、分辨率、原点、轴方向、尺度和可通行区域，再构建相同坐标的 Gazebo 场地。只替换 world/model 资源，保留当前话题和 map→odom→base_link 契约。用至少三个地标验证场景与规划地图对齐，不能只看外观。
然后添加雷达/点云传感器与外参，核对现有点云话题、frame 和时间戳。选择真值定位或真实定位算法二者之一作为唯一定位/TF 来源。
最后单独接入现有全局规划、路径可视化，再接跟踪控制；先限速、验证停机，不在本阶段修改规划或 MPC 算法。
若要验证避障、接触、加速度和实车性能，必须先替换为经过标定的动力学/碰撞模型并增加对应验收。

参考实现依据：[Humble / Fortress 配对](https://index.ros.org/r/ros_gz/)，
[Fortress VelocityControl](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/velocity_control/VelocityControl.cc)，
[Fortress OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/odometry_publisher/OdometryPublisher.cc)。
