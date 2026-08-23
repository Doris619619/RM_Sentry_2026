<!-- 此文件用于说明 ROS2 左 MID360 Point-LIO 与 HDL 定位链的真机网络前置条件、启动顺序和验收标准。 -->

# ROS2 定位链真机启动与验收

本说明对应 feat/20260823-ros2-localization，第一版只验收左 MID360：

左 MID360 LiDAR + 左 MID360 IMU → Point-LIO → odom → base_link → HDL → map → odom

右 MID360 融合默认关闭。没有可靠外参时，不得打开 enable_dual_lidar_fusion。

## 1. 启动前必须满足的条件

1. 主机必须保留用于 SSH 的管理网地址，并额外拥有 Livox 驱动 JSON 中 host_net_info 所配置的数据网地址。当前仓库默认是 192.168.1.50。
2. 左 MID360 的地址必须可达：192.168.1.3。右 MID360 为 192.168.1.105，但第一版不依赖它。
3. 外部全局地图 PCD 必须已经准备好，且调用 launch 时提供其绝对路径；地图文件不得提交到仓库。
4. ROS2 Humble 工作区必须已经构建并 source。

当前主机仅检测到 192.168.80.128，未检测到 JSON 所需的 192.168.1.50 数据网地址。因此在配置第二块网卡或为传感器网卡增加该地址前，Livox 驱动不会收到左 MID360 的 LiDAR 与 IMU 数据。

先执行：

~~~bash
ip -brief address
ping -c 3 192.168.1.3
source /opt/ros/humble/setup.bash
cd /home/liangys/RM_Sentry_2026/ros2_ws
source install/setup.bash
~~~

地址与传感器网络由现场网络规划决定；修改主机 IP、网卡路由或雷达 IP 前，必须确认不会中断管理网络和其他设备。

## 2. 构建与传感器数据检查

~~~bash
source /opt/ros/humble/setup.bash
cd /home/liangys/RM_Sentry_2026/ros2_ws
colcon build --parallel-workers 1
source install/setup.bash
~~~

先单独启动驱动与点云处理器，确认左雷达数据存在。若现场已经运行同名节点，不能再启动第二份。

~~~bash
ros2 launch livox_cloudpoint_processor dual_mid360_cloud.launch.py \
  enable_dual_lidar_fusion:=false
~~~

另一终端检查：

~~~bash
source /opt/ros/humble/setup.bash
source /home/liangys/RM_Sentry_2026/ros2_ws/install/setup.bash
ros2 topic info -v /livox/lidar_192_168_1_3
ros2 topic info -v /livox/imu_192_168_1_3
ros2 topic echo --once /livox/lidar_192_168_1_3
ros2 topic echo --once /livox/imu_192_168_1_3
~~~

两个输入话题都必须有发布者。左 LiDAR 的类型必须是 livox_ros_driver2/msg/CustomMsg，并检查 timebase 与每个点的 offset_time 可用、连续；IMU 时间戳也必须持续单调递增。

## 3. 启动完整定位链

以下命令由统一 bringup 启动 Livox 驱动、点云处理、Point-LIO、地图服务器和 HDL：

~~~bash
source /opt/ros/humble/setup.bash
source /home/liangys/RM_Sentry_2026/ros2_ws/install/setup.bash
ros2 launch hdl_localization sentry_localization_bringup.launch.py \
  globalmap_pcd:=/绝对路径/地图.pcd \
  enable_dual_lidar_fusion:=false
~~~

现场参数可从 launch 覆盖，包括左侧输入话题、odom_frame、base_frame、LiDAR/IMU QoS、IMU 缓存时长、HDL TF 查询容差和 NDT 分辨率。第一版的稳定契约是：

| 项目 | 约定 |
| --- | --- |
| Point-LIO 输入 | /livox/lidar_192_168_1_3、/livox/imu_192_168_1_3 |
| 局部里程计 | /point_lio/odometry，odom → base_link |
| HDL 输入点云 | /filted_topic_3d |
| 最终定位 | /localization/odometry，map → base_link |
| 唯一全局 TF | HDL 发布的 map → odom |

HDL 默认 use_imu=false，仅使用 Point-LIO 里程计作运动预测。它在扫描时刻查询 T_odom_base，并按 T_map_odom = T_map_base × inverse(T_odom_base) 发布 map → odom；如果 TF 缺失或超时，应该拒绝该次 TF 更新并给出中文诊断，而不是发布错误的 map → base_link。

## 4. 最终验收命令

~~~bash
source /opt/ros/humble/setup.bash
source /home/liangys/RM_Sentry_2026/ros2_ws/install/setup.bash
ros2 topic hz /point_lio/odometry
ros2 topic hz /localization/odometry
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
rviz2
~~~
