<!-- 文件用途：记录六项后续仿真的实施顺序、当前状态和可复现验收边界。 -->
# 自主导航仿真实施记录

基线：已合并 PR #10，4210fa7。分支：feat/20260926-sentry-sim-autonomy。

| 顺序 | 项目 | 状态 |
|---|---|---|
| 1 | 现有 MPC 自动跟踪闭环 | 实施中，尚未验收 |
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
不会覆盖第一、二阶段构建。新机器需要先完成仓库已有 OCS2 依赖构建，不能假定该目录存在。

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
  下游原有暂停保护继续保留，真实暂停/恢复仍待故障注入验收。
- 原规划器每帧点云都会触发重规划，180 秒出现 155 条轨迹，导致跟踪反复重新计时。
  现在点云先更新占据，只在当前路径被阻挡时触发；保留跟踪器 5 秒主动刷新和换目标能力。
- 原 mu=20 的低速控制在距原始墙面约 1.3 米处趋近停止，180 秒不能到达。
  仿真使用 mu=1 后首条完整路线通过：45.31 秒、终点误差 0.1130 m、最大横向偏差 0.0319 m、
  最高速度 0.3244 m/s、占据样本 0。见 evidence/closed-loop-cost-tuned.json。
- 9 项隔离保护测试通过，见 evidence/guard-contract-tests.log；合成输入仅用于异常契约，
  实际运动使用 Gazebo 反馈。
- 五组各三次及 310 秒性能测试正在执行，不能将单路线成功视作全部阶段通过。

GUI 布局辅助只调整当前 launch 的 Gazebo/RViz 子窗口，不占用鼠标；
低刷新率 RViz 与可见小窗口用于减少软件渲染负载。

规划器初速度坐标检查另发现：原适配器直接把 base_link 线速度当作 map 速度使用。已补充按定位航向旋转一次；默认零航向路线不变，非零航向闭环验证待执行。
