<!-- 文件用途：记录第二步接口解耦的实际验证、失败样例与交付边界。 -->
# 第二步导航接口验收记录

日期：2026-11-09。环境：VMware Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress，使用单独工作区构建，复用已有 OCS2/HPIPM 依赖。原工作区主分支没有修改。

## 已通过的接口验证

- 六包构建通过：`waypoint_generator`、`trajectory_generation`、`trajectory_tracking`、`decision_node`、`sentry_gazebo`、`sentry_bringup`。编译器仍报告已有算法中的未使用变量/参数告警；没有构建失败。
- 规划/跟踪测试及外部接口 launch test 的 `colcon test-result` 汇总：36 tests，0 errors，0 failures，0 skipped。
- 地图配置 Python 测试 4 项通过：原地图与外部目录一致、双方元数据不一致拒绝、尺寸与范围不一致拒绝、manifest 原点不一致拒绝。
- 新旧地图与动态保护距离 Python 测试 2 项通过：0.05/0.1 m 分辨率对应同样的 0.40 m 保护距离、过期点清除、新地图范围外拒绝。
- 外部接口测试通过：使用 best-effort 合成 `map/base_link` 里程计和点云，重映射主要话题后运行真实规划器/SQP MPC；检查非空轨迹、solver 成功、非零速度、错误目标帧取消以及取消后迟到的 replan 请求不能恢复旧目标。该测试没有模拟车辆移动。
- 地图 manifest 的 9 项 SHA-256 与本地文件一致，包含原始 PNG/压缩网格和适配配置；Python 语法及 `git diff --check` 通过。

外部 launch 测试直接通过该包生成的 CTest 运行。对整个源码树执行 `colcon test --packages-select sentry_bringup` 会寻找未在本次六包 overlay 构建的 Point-LIO 等包；这是包发现范围问题，复现命令使用 [接口说明](navigation_interfaces.md) 中的定向 CTest，不声称整个生产栈测试通过。

## 新地图实际闭环

使用 `gazebo_navigation.sh`：RMUC2024 原始 OCC/BEV/Topo、GPU 雷达、世界系点云、Gazebo 真值里程计、理想全向底盘、真实规划器、真实 SQP MPC、原有安全门。`full_system=false`，不使用 NDT、接触动力学、决策或 MCU。

最终代码完成约 3 米往返：

| 记录 | 起点（实测）→目标（m） | 结果 | 终点误差 | 墙钟耗时 | 最大实测速度 |
| --- | --- | --- | --- | --- | --- |
| goal-3 | `(5.000,8.000)` → `(7.95,7.55)` | 到达 | 0.1147 m | 12.34 s | 0.3113 m/s |
| goal-4 | `(7.8366,7.5674)` → `(4.95,7.55)` | 到达 | 0.1124 m | 11.66 s | 0.3126 m/s |

两次之间没有重置或瞬移。分别记录 616、583 个里程计样本，静态占用区样本均为 0；各有 3 条通过安全门的轨迹，违规参考轨迹计数为 0。里程计帧为 `map/base_link`，点云帧为 `map`；到达后测得速度为 0，安全门确认稳定至少 0.5 秒。

本轮早期短距离 smoke test（goal-1）也成功：`(5,8)` → `(4.95,7.55)`，误差 0.1163 m，169 个样本，无占用区样本。随后加入了“取消目标后忽略迟到 replan”的接口保护并重新构建，以上 goal-3/4 使用最终源码。

## 保留的失败样例

goal-2：从 `(4.9414,7.6660)` 请求 `(15.45,8.35)`。候选参考轨迹被安全门以 `reference intersects occupied footprint grid` 拒绝；没有轨迹获准执行，8985 个样本的速度均为 0、位置不变。早期 probe 等待 180 秒后报告失败；当前 probe 会在收到终止状态 `stopped` 时提前失败并保存记录。

因此，本次**不能宣称新地图任意航点可达、跨场路线通过或 benchmark 成功率合格**。应在第三步继续定位拓扑/平滑/参考轨迹与占用检查的差异，再固定场景和起终点集计算成功率。本次没有放宽碰撞阈值来放行该路径。

## 交付与后续边界

- 独立入口、可重映射的定位/点云/目标/轨迹/速度接口、共享地图配置目录及校验已交付；输入契约见 [导航接口说明](navigation_interfaces.md)。
- 动态障碍栅格计算仍在原规划/跟踪地图类中。接队友在线障碍地图时，需要按约定适配高度、膨胀、更新与清除语义；尚未实现通用 OccupancyGrid/在线地图服务。
- 未验收新场景 NDT 精度、物理碰撞与坡道、真实 LIO/实车，未制作正式 demo 视频，未开展预先固定样本集的批量 benchmark。
- 机器可读结果、失败原因、首尾样本和原始轨迹文件 SHA-256 见 [测试结果](navigation_step2_results.json)。完整原始样本和构建/仿真日志保存在开发机 `D:\CodexData\sentry-step2-20261109`，虚拟机副本为 `/home/liangys/sentry-step2-evidence`；这些本机路径不是远程下载链接。
- 原地图历史 15/15 验收保持独立，不能并入本次新地图结果。仿真在验证结束后已完整退出。
