<!-- 文件用途：记录原始地图来源、ROS2 适配边界以及资源清单职责。 -->
# RMUC2024 上游场景

此目录保存上游 HIT 开源仓库 `zzt-180/sentry_planning` 的 RMUC2024 默认仿真地图，版本固定在 `b9349d2ada3fe0299ccae9d1e483d5f3b79b1cc8`。

- 默认规划使用上游 ROS1 仿真 launch 原本配套的 `occ2024low.png`、`bev2024low.png` 和 `occtopo2024low.png`。三张原图未经重绘或重采样，像素大小为 301×161，分辨率为 0.1 m，坐标原点为 `(0, 0)`。
- `occtopo2024low.png` 是开源仓库提供的车道拓扑骨架。ROS2 规划器加载它用于拓扑搜索；RViz 也直接把该像素骨架显示为青色线段。
- `source/rmuc_static.world` 和压缩的 `source/RMUC2024_field_base_link.STL.gz` 来自上游 Gazebo 目录。网格压缩仅用于减少源码体积；构建时会验证并原样解压原始 SHA-256 和 Git blob SHA。
- `map_metadata.yaml` 保留上游仿真 launch 使用的坐标、分辨率、车辆半径和高度编码参数。ROS2 将范围统一为图像实际覆盖的 30.1×16.1 m，规划器和跟踪器共享配置；不把图外区域补成自由空间。
- 默认机器人位姿 `(5, 8)` 复用了上游仿真启动位置。巡逻点选在原始骨架附近的可通行区域。

上游使用 ROS1 Noetic / Gazebo Classic。当前包只复用其赛场网格与地图，将静态场地模型接入现有 ROS2 Humble / Gazebo Fortress 世界；机器人、ROS2 雷达桥接、定位、规划和控制仍由本项目启动。

文件路径、Git blob SHA 和上游 MIT 许可证见 `sources.json` 与 `LICENSE`。

`map_manifest.json` 用于选择配对文件、声明坐标范围以及记录原图与适配配置的 SHA-256。
其中 `map_metadata.yaml` 的哈希随本地适配更新，上游三张 PNG 和场地网格的哈希保持原值。
