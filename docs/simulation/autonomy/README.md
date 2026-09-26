<!-- 文件用途：记录六项后续仿真的实施顺序、当前状态和可复现验收边界。 -->
# 自主导航仿真实施记录

基线：已合并 PR #10，4210fa7。分支：feat/20260926-sentry-sim-autonomy。

| 顺序 | 项目 | 状态 |
|---|---|---|
| 1 | 现有 MPC 自动跟踪闭环 | 已通过 15 次实际闭环及 90° 航向复核 |
| 2 | 安全停车、控制权及异常恢复 | 待验收 |
| 3 | 移动中换目标与动态障碍重规划 | 待验收 |
| 4 | 实际定位算法与真值误差对照 | 待实施 |
| 5 | 通用底盘物理与场地验证 | 待底盘参数；可先做明确标注的通用模型 |
| 6 | 决策、控制、定位与模拟通信完整联调 | 待实施 |

每项通过后记录命令、版本、数据和限制；没有实测的项不写为完成。
继续使用 ROS domain 26、仿真时钟，保留第一、二阶段入口。
本阶段默认仅连接 Gazebo，禁止物理 MCU 串口输出。

## 当前实现与命令

在 Ubuntu 桌面终端：

~~~bash
cd /home/liangys/RM_Sentry_2026
bash scripts/gazebo_autonomy.sh build
bash scripts/gazebo_autonomy.sh
~~~

另开终端手动接管：`bash scripts/gazebo_autonomy.sh keyboard`。
切回自动：`bash scripts/gazebo_autonomy.sh auto`，然后在 RViz 重新点击目标。
紧急停止：`bash scripts/gazebo_autonomy.sh stop`；结束全部仿真用启动终端 Ctrl+C。
切回自动不会恢复旧目标。自动到点当前沿用全向平移语义，保持机器人自身航向；
目标验收是位置误差 <=0.15 m、实测线速度 <=0.03 m/s、角速度 <=0.05 rad/s，连续稳定 0.5 秒。

构建复用本机已验收 OCS2、HPIPM、消息依赖所在的 install_closure_fix，再独立构建五个包，
不会覆盖第一、二阶段构建。新机器需要先按 ros2_ws/README.md 的固定源码 bootstrap 和完整构建步骤准备依赖，
再显式指定标准安装目录；本机 install_closure_fix 只是已验证的可复用构建：

~~~bash
export SENTRY_DEPENDENCY_SETUP=/absolute/path/RM_Sentry_2026/ros2_ws/install/local_setup.bash
bash scripts/gazebo_autonomy.sh build
bash scripts/gazebo_autonomy.sh
~~~

新机器的完整从零构建尚未作为本次验收数据，不能假定归档 underlay 目录随仓库存在。

## 控制与参数

唯一输出链为 MPC → hit_bridge → /sim/auto_cmd_vel → autonomy_guard → /cmd_vel → 原限幅保护 → Gazebo。
键盘只发布 /sim/manual_cmd_vel，/sim/control_mode 显式选择 auto、manual 或 stop。
/goal 先经过保护节点取消旧轨迹，再转发 /sim/planner_goal。
规划结果经 /sim/planned_trajectory 校验后才转发 /global_trajectory，消息结构保持不变。

仿真配置：期望速度 0.3 m/s，规划最大速度 0.45 m/s，加速度 0.35 m/s²，MPC 输出限制 0.45 m/s，
最下游仍为 0.5 m/s、0.5 rad/s、0.5 秒保护。保护节点解析多项式理论导数极值，
拒绝超出 0.5 m/s 或 0.4 m/s² 的参考，不通过仅截断输出追赶过快轨迹。

低速场景的 MPC 障碍软代价 mu 使用 1.0；生产 tracking.yaml 的 20.0 保留。
这是仿真专用调参，仍需实际路线占据与动态障碍验收，不能用调参代替碰撞检查。

## 已发现的问题和证据

- 首次运行在 0.2 秒时钟停滞判断触发后安全停车。自主保护统一采用 0.5 秒停滞和控制输出超时判据；
  下游原有暂停保护继续保留，真实暂停/恢复已通过第二项故障注入验收。
- 原规划器每帧点云都会触发重规划，180 秒出现 155 条轨迹，导致跟踪反复重新计时。
  现在点云先更新占据，只在当前路径被阻挡时触发；保留跟踪器 5 秒主动刷新和换目标能力。
- 原 mu=20 的低速控制在距原始墙面约 1.3 米处趋近停止，180 秒不能到达。
  仿真使用 mu=1 后首条完整路线通过：45.31 秒、终点误差 0.1130 m、最大横向偏差 0.0319 m、
  最高速度 0.3244 m/s、占据样本 0。见 evidence/closed-loop-cost-tuned.json。
- 12 项隔离保护测试通过，见 evidence/guard-terminal-fixed-tests-rerun.log；合成输入仅用于异常契约，
  实际运动使用 Gazebo 反馈。
- 最终源码五组各三次 15/15 通过；空间审计 15/15 通过，90° 航向复核通过。310 秒双窗口性能通过。第二项安全验收见下文；第三至六项仍待验收。

GUI 布局辅助只调整当前 launch 的 Gazebo/RViz 子窗口，不占用鼠标；
低刷新率 RViz 与可见小窗口用于减少软件渲染负载。

规划器初速度坐标检查另发现：原适配器直接把 base_link 线速度当作 map 速度使用。已补充按定位航向旋转一次；90° 初始航向实测通过，终点误差 0.1142 m、最大横向偏差 0.0345 m，见 evidence/closed-loop-clearance-yaw90.json。


## 扩大回归后发现的边界问题

第一轮 15 次实测在第 11 次发现实际占据区样本；归档
evidence/closed-loop-15-final.json，不能算该轮通过。对应重规划多项式也有占据区采样，
所以增加原生产占据图的参考轨迹及 MPC 预测检查，并在自主配置增加 0.45 m 规划余量。
演示终点改用有充分净空的 (-3.219, 2.146)，原图、元数据与第二阶段入口保留。
规划 re-anchor 在跳到前方路径点前必须检查连线，防止裁剪跨墙角。

反向路线在拓扑图端点连接失败，已增加双向回归。端点连接所有可见守卫，
可见性采样包含两端且间隔不超过半栅格；零距离不再除零，Dijkstra 最小代价保留浮点精度。
修复后独立反向实测通过，见 evidence/closed-loop-reverse-fixed.json；
9 项规划回归通过，见 evidence/topo-after-fix-correct-overlay.log。
早期 topo-after-fix.log 误加载旧 underlay 动态库，属于测试启动错误；正确 overlay 的 ldd 已核验。

ROS2 适配器已将当前位姿传入旧拓扑采样器的 odom_position；最终 closed-loop-final-source-15.json 包含此修正，15/15 通过。

实际运动原始采样按约 12 Hz 保存；audit_closed_loop.py 对相邻实测位置按 <=0.02 m
插值检查，并对每一条已接受多项式按解析速度界限控制 <=0.02 m 间隔检查。
插值检查不冒充额外传感器测量。


## 第一项最终验收

最终批量 evidence/closed-loop-final-source-15.json：15/15 到点，累计实际路线测试 577.16 秒；
最大位置误差 0.1172 m、最大横向偏差 0.0374 m、最大速度 0.4202 m/s。
所有接受轨迹及实测位置插值通过原占据图检查，见 evidence/closed-loop-final-source-audit.json。
最终 90° 航向实测 49.42 秒，误差 0.1138 m，见 evidence/closed-loop-final-yaw90.json。
源文件与实际加载程序哈希见 evidence/step1-final-source-manifest.json。
第二项进程卡死、暂停、接管与重启实际测试尚未计入本结论。


## 第二项安全验收与重启

9 项实际故障注入、14 项保护契约、3 项启动契约、原场景 34 项回归通过。
完整重启使用 `bash scripts/gazebo_autonomy.sh restart`；停止运动仍使用 stop，
退出全部进程使用启动终端 Ctrl+C。原生时间回拨将锁停并要求完整重启，不能用新目标解锁。
详细指标及失败记录见 [验收报告](验收报告.md)。

第三项移动中重规划已通过：换目标、真实箱体绕行、箱体位移和封路停车。
实际定位、底盘动力学和完整 MCU 链路仍待第四至六项验收。


## 使用实际 NDT 定位

`SENTRY_LOCALIZATION=ndt bash scripts/gazebo_autonomy.sh` 使用估计定位控制；
`SENTRY_LOCALIZATION=ndt bash scripts/gazebo_autonomy.sh restart` 可切换已有仿真。
真值仍在 /sim/ground_truth/odometry，仅供评估；运行节点不消费它。
本配置验证有初始位置先验的平面导航，不代表 Point-LIO 或未知位置全局重定位已经通过。
第四项往返和故障恢复已通过；第五、六项仍待验收。

第五项通用底盘物理验收已完成：8 项测试通过，包括真实重力、摩擦、撞墙、制动和 5° 坡道。
这仍是通用模型，不能作为真车轮系和电机动力学验收；第六项完整系统联调待完成。
