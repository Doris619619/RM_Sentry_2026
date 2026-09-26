<!-- 文件用途：记录第二阶段实施入口、分阶段状态和复现方式。 -->

# 第二阶段：场地、点云与规划展示

当前正在进行 M0 点云可行性验收，尚未宣称整个第二阶段通过。
第一阶段入口保持不变；两阶段使用相同的互斥启动锁，不能同时运行。

在 Ubuntu 桌面终端运行：

```bash
bash /home/liangys/RM_Sentry_2026/scripts/gazebo_planning.sh probe
```

探针场景使用真实 Gazebo gpu_lidar（360×4，5 Hz，0.2～10 米）。
雷达位于 base_link 上方 0.7 米；x=3 米处的墙面用于独立测距验收。
另开终端以同一脚本的 keyboard 参数控制，空格停车、Esc 退出。
第一个终端 Ctrl+C 停止 Gazebo、桥接、适配和 RViz。

源码修改后使用同一脚本的 build 参数，只构建四个相关包，并发 2。
独立目录为 build_sentry_sim2、install_sentry_sim2、log_sentry_sim2。

点云链路：Gazebo 原生点云 → /sim/lidar/native → /sim/lidar/points（lidar_link）
→ /filted_topic_3d（base_link）与 /aligned_points（map）。
原始测量时间戳保留，缺少对应时间 TF 的点云等待最多 0.5 秒后丢弃。
原始点云可以包含没有击中障碍的无穷远值；过滤后输出仅含有效回波。
证据目录中的 JSON 用于机器可读验收记录，日志记录实际过程，不手工插入注释。

## M0 实测结果

真实 Gazebo 点云测距、平移和转向检查通过。x=3 米墙面的误差最大 0.0101 米。
40 秒初步性能记录：实时率 0.9999，点云 5 Hz，CPU 合计约 340.6%（100%=一个核），RSS 合计峰值 1500 MiB。
Gazebo 与 RViz 图形显示已检查，截图见 evidence/m0-lidar.png。最终完整场景仍需重新进行五分钟验收。
首次运行发现 Gazebo 点云含整数 ring 字段，已改为结构化 XYZ 提取，避免错误假设全部字段为浮点类型。
