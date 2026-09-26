<!-- 文件用途：说明第二阶段场地、真实点云和规划展示的运行方法、接口与验收边界。 -->

# 第二阶段：地图、点云与规划展示

本阶段在 RM_Sentry_2026 中连接真实 Gazebo 传感器和现有全局规划器。
机器人仍由键盘控制，没有启动自动跟踪、真实定位、决策或 MCU。
**当前轨迹段间连续性存在未解决问题，不能据此宣称自动导航已经可用。** 详见 [问题记录](KNOWN_ISSUES.md)。

## 在 Ubuntu 桌面启动

先关闭第一阶段或此前的仿真，在 Ubuntu 终端运行：

```bash
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh
```

Gazebo 显示地图对应的简化场地，RViz 显示占据地图、蓝色机器人、橙色点云、紫色目标箭头和规划结果。
在 RViz 顶部选择 **2D Goal Pose**，在灰色可通行区域按住左键拖出一个方向后松开。
目标发到 /goal，当前规划器使用目标位置，最终通过原有 Marker 话题显示结果。
机器人不会自动跟随路径；路径出现与自动到点是两个不同的验收项目。

另开一个 Ubuntu 终端：

```bash
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh keyboard
```

等待“控制接口已连接”。W/S 前后、A/D 横移、Q/E 旋转；空格停车，Esc 退出键盘。
平移 0.2 m/s、转向 0.3 rad/s；继承 0.5 m/s、0.5 rad/s 限幅和 0.5 秒无指令停车。
第一个终端按 **Ctrl+C** 停止整个仿真。VMware 捕获鼠标时按 **Ctrl+Alt** 返回 Windows。
这是无碰撞动力学的理想平面模型，手动操作不要穿越障碍后把它当作碰撞通过证据。

## 模式与构建

```bash
# 小场景雷达探针：已知 x=3 米墙面，用于真实测距
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh probe

# 地图与点云，不启动规划器
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh map

# 修改源码或资源后重新构建
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh build
```

构建仅涉及 sentry_gazebo、sentry_bringup、trajectory_generation、waypoint_generator，并发最多 2。
专用输出目录为 build_sentry_sim2、install_sentry_sim2、log_sentry_sim2；不清理其他构建目录。
第一阶段脚本 gazebo_basic.sh 保留，两个入口共用同一用户/domain 的互斥锁。

## 地图与模型

采用规划器原有 occfinal.png、bevfinal.png、occtopo.png 和共享元数据，不修改源地图。
分辨率 0.05 米、400×400、20×20 米，下界 (-13.394, -12.079)。
出生点 (-0.919, -4.454)、航向零；基线目标 (-2.719, 1.946)。
以原始占据值大于 10 的区域生成障碍，不把机器人安全膨胀区再次挤出。
2162 个合并长方体组成一个网格，保留原图中的稀疏点和不规则边缘，不做美观化平滑。
统一高度 1 米只用于二维轮廓展示和观测，不表示真实地形、坡道或多层通行结构。

生成脚本、网格和 config/map_manifest.json 都在 sentry_gazebo 内。
清单记录五个源文件的 SHA-256、占据格覆盖和地标。启动时验证安装后的规划地图哈希，
不一致则退出，避免静默使用与规划器不同的场地。

更换地图后，在确认元数据和自由区出生点后重新生成并构建：

```bash
python3 /home/liangys/RM_Sentry_2026/ros2_ws/src/sentry_gazebo/scripts/map_assets.py
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh build
```

当前出生点属于本次基线，换图时必须重新核对；不能仅运行转换脚本便认定新地图已验收。

## 接口

| 话题 | 类型与坐标 |
|---|---|
| /sim/lidar/native | bridge 内部 PointCloud2；Gazebo 真实 gpu_lidar 输出，含 ring 等字段 |
| /sim/lidar/points | PointCloud2，lidar_link；保留原始测量时间戳和回波 |
| /filted_topic_3d | PointCloud2，base_link；去除非有限值、无回波和量程外点 |
| /aligned_points | PointCloud2，map；使用测量时刻 TF 转换 |
| /localization/odometry | Odometry，map / base_link；Gazebo 真值，50 Hz 仿真时间 |
| /goal | PoseStamped，map；RViz 目标输入 |
| /global_trajectory | 原 TrajectoryPoly；未改消息字段或坐标语义 |
| /sim/map | OccupancyGrid，map；供 RViz 显示源地图，不是传感器数据 |
| /sim/robot | Marker，map；由真实位姿驱动的机器人显示 |

雷达安装于 base_link 上方 0.7 米，360×4 点、5 Hz、量程 0.2～10 米，垂直角 ±0.1 rad。
此简化雷达不模拟 MID360 扫描特性。缺少测量时刻 TF 时最多等待 0.5 秒，随后丢帧；
不使用最新 TF 兜底。原始回波允许无穷远值，过滤后的两路点云只含有效返回点。

TF 为 map → odom → base_link → lidar_link，最后一段为固定外参。
地图定位与传感器外参各有一个静态 TF 发布者，这是不同边的正常发布，不是重复定位。
ROS_DOMAIN_ID=26、ROS_LOCALHOST_ONLY=1，所有运行节点显式 use_sim_time=true。
ROS /clock 和定位各一个发布者；不同时启动 fixture、HDL 或 Point-LIO。

## 验收、图形与汇报

保持已验证的 llvmpipe 软件渲染、Ogre2 和 XWayland；没有修改 VMware 资源或用户全局图形驱动。
按用户要求，Ubuntu 自动熄屏已设为从不，空闲变暗关闭；与仿真源码功能无关。
QA 额外安装 wmctrl 用于并排窗口，正常启动不依赖它。录像使用已有 GStreamer，
成片在 Windows 已有 FFmpeg 中编码，不在虚拟机新增视频编码套件。

[验收报告](ACCEPTANCE.md) 区分通过项、未通过项和环境受限的检查。
[队内五页汇报提纲](TEAM_REPORT.md) 可直接用于队会。
[第三阶段交接](HANDOFF.md) 说明自动跟踪前必须解决的问题。
evidence 中的 JSON 是机器可读验收结果；before、diagnostic 文件是排障历史，不能当作最终通过结果。
