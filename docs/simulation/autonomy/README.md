<!-- 文件用途：提供六项自主仿真的当前状态、分层入口、复现命令和验收材料索引。 -->
# 自主导航仿真

基线为已合并 PR #10（4210fa7），实施分支为 feat/20260926-sentry-sim-autonomy。
六项已分别完成实际仿真验收；真车、电控硬件和真实轮系辨识仍在本轮边界之外。

| 顺序 | 内容 | 结果 |
|---|---|---|
| 1 | 既有 MPC 自动跟踪 | 五组各三次，15/15 到点；90° 航向复核通过 |
| 2 | 安全异常、接管和重启 | 9 项真实故障、完整重启、原场景 34 项回归通过 |
| 3 | 移动中重规划 | 换目标、真实箱体绕行、箱体位移、封路停车通过 |
| 4 | 实际定位算法 | CPU HDL/NDT 往返、误差和故障恢复通过 |
| 5 | 通用底盘动力学 | 8 项真实重力、摩擦、制动、碰撞和坡道测试通过 |
| 6 | 完整系统与模拟串口 | 真实策略/MCU 往返、故障、暂停、310 秒双 GUI、完整重启通过 |

## 启动

在 Ubuntu 22.04 / ROS 2 Humble / Fortress 的桌面终端：

```bash
cd /home/liangys/RM_Sentry_2026
bash scripts/gazebo_full.sh build
bash scripts/gazebo_full.sh
```

另开终端，按 [完整系统接口](完整系统接口.md) source 环境并发送裁判场景。
完整系统默认比赛未开始，车辆不会自行移动。
通过 auto 模式与 patrol 场景，真实策略节点才会发出巡逻目标；
supply 场景通过真实串口解码低血量，触发返补给。

- 手动接管：`bash scripts/gazebo_full.sh keyboard`。
- 停止运动：`bash scripts/gazebo_full.sh stop`。
- 完整退出：启动终端 Ctrl+C。
- 完整重启：`bash scripts/gazebo_full.sh restart`。
- 故障或手动接管后重新开始巡逻：先发送 stop 裁判场景，再选 auto 并发送 patrol。
- 原生时间回拨会锁停，必须完整重启；不会用旧目标继续运动。

## 分层入口和构建边界

| 入口 | 用途 |
|---|---|
| `bash scripts/gazebo_autonomy.sh` | 理想平面底盘、真值定位、实际 MPC 与点云，便于逐层回归 |
| `SENTRY_LOCALIZATION=ndt bash scripts/gazebo_autonomy.sh` | 理想平面底盘、实际 CPU NDT，保留第四项已验收参数 |
| `bash scripts/gazebo_physics.sh` | 独立通用力驱动底盘、真实碰撞与坡道，手动验证 |
| `bash scripts/gazebo_full.sh` | 实际策略、NDT、规划、MPC、真实 MCU PTY 和力驱动底盘 |

入口共用独占锁、ROS domain 26 和仿真时间，不能同时启动两套。
GUI 布局只调整当前 launch 的窗口，不占用鼠标。
第一、二阶段入口仍保留，见上级目录说明。

构建复用本机既有 OCS2/HPIPM 依赖 install_closure_fix，并独立构建六个包。
新机器需按 ros2_ws/README.md 准备依赖，然后指定实际安装目录：

```bash
export SENTRY_DEPENDENCY_SETUP=/absolute/path/RM_Sentry_2026/ros2_ws/install/local_setup.bash
bash scripts/gazebo_full.sh build
```

从零准备新机器尚未作为本轮实测，不能假设归档 underlay 目录随 Git 分发。

## 参数和材料

分层自主入口期望速度 0.3 m/s、规划/MPC 上限 0.45 m/s。
完整系统期望速度 0.25 m/s、上限 0.35 m/s；最下游仍保留 0.5 m/s、
0.5 rad/s 和 0.5 秒保护。仿真 MPC 障碍软代价为 1，生产配置的 20 保留。
独立占据与动态障碍检查仍约束参考和预测，不以调参替代碰撞检查。

完整系统使用 0.3 m KDTREE NDT、0.05 m 下采样、真实 360×4/5 Hz 雷达。
公共定位唯一来自估计结果，真值仅供验收；目标位置到达不代表目标朝向跟踪已完成。

- [六项验收报告、失败记录和原始证据](验收报告.md)
- [完整系统命令、接口与安全约束](完整系统接口.md)
- [五页队会汇报和真车前交接](队会汇报与交接.md)
- [95 秒真实双窗口演示](full-demo.mp4)

场地为规划地图轮廓挤出，底盘为通用全向力模型；未连接电控板。
不能据此宣称真实比赛场地、真实轮胎电机、未知位置重定位或完整比赛行为已经验收。
