# HIT 原开源 RMUC2024 场景

默认 ROS2 仿真现已使用上游 [`zzt-180/sentry_planning`](https://github.com/zzt-180/sentry_planning) 仓库中的 RMUC2024 场景资源，固定到提交 `b9349d2ada3fe0299ccae9d1e483d5f3b79b1cc8`。

## 加载资源

- ROS1 仿真 launch 默认选择 `occ2024low.png`、`bev2024low.png`、`occtopo2024low.png`。ROS2 继续使用这一套配对地图，没有重新绘制或重采样；原始文件和 SHA 记录在 [`sentry_2024` 场景目录](../../ros2_ws/src/sentry_gazebo/maps/sentry_2024/README.md)。
- OCC、BEV 和 Topo 的尺寸为 301×161 像素、分辨率 0.1 m、地图原点 `(0,0)`。ROS2 规划器和跟踪器读取同一份场景元数据。
- Gazebo 使用上游 `RMchangdi1230` 的 `base_link.STL` 场地网格。构建时验证源文件哈希并逐字节原样解压，再由 SDF 世界加载。世界仍保留本项目的哨兵模型、GPU 雷达、ROS2 topic bridge、NDT/规划/跟踪节点。
- RViz 的青色线由原始 `occtopo2024low.png` 像素转换为线段；规划器也直接加载该图运行拓扑搜索。
- 初始机器人位姿采用上游仿真 launch 使用的位置 `(5,8)`。巡逻途经点布置在原始骨架附近的自由区域。

## 上游与适配边界

上游仓库是 ROS1 Noetic 和 Gazebo Classic 工程。ROS2 当前启动链没有启动上游 ROS1 launch 或加载 Classic 插件；它只导入原始 OCC/BEV/Topo 图片、RMUC2024 静态场地网格和原始场景文件，并通过 ROS2 SDF 世界加载。底盘、雷达、NDT、导航、控制、菜单和消息桥接沿用本仓库 ROS2 代码。

运行默认真值定位和理想底盘：

```bash
./sim.sh
```

切换到物理碰撞与 NDT 定位：

```bash
SENTRY_PHYSICS=true SENTRY_LOCALIZATION=ndt ./sim.sh restart
```

完整决策与模拟 MCU 链路：

```bash
SENTRY_PHYSICS=true SENTRY_LOCALIZATION=ndt SENTRY_FULL_SYSTEM=true ./sim.sh restart
```

完整场景源数据的 MIT 许可证随地图文件保存在场景目录中。
