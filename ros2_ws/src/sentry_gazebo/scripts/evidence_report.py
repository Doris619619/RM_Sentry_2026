#!/usr/bin/env python3
"""Regenerate the Chinese acceptance report from actual recorded evidence."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
DOC=ROOT/'docs/simulation'
E=DOC/'evidence'
motion=json.loads((E/'motion.json').read_text())
checks={r['name']:r for r in motion}
assert all(r['passed'] for r in motion)
life=json.loads((E/'keyboard-lifecycle.json').read_text())
assert all(r['passed'] for r in life)
st=json.loads((E/'stability.json').read_text())
assert st['duration_wall']>=300 and st['all_alive']
assert all(len(s['processes'])==6 for s in st['samples'])
disk=json.loads((E/'disk-after.json').read_text())
sizes=json.loads((E/'package-size.json').read_text())
raw=checks['raw_state_matches']
report=f"""# 第一阶段仿真验收报告

日期：2026-09-26。开发目标为 **/home/liangys/RM_Sentry_2026**，分支 **codex/sentry-sim-part1**，基于 459c0a6。
交付的是本仓库的基础导航仿真，不是同学 rm_sentry_nav 的复现。
生产启动、fixture 行为、规划/跟踪算法未修改。

## 结果

最终版本的 {len(motion)} 项现场运动/接口检查和 {len(life)} 项键盘生命周期检查全部通过。
验证时 Gazebo GUI 实际显示在 Windows VMware 的 Ubuntu 桌面，包含单辆机器人、简单平地、坐标箭头和远处展示障碍。

| 验收项 | 实测结果 | 原始证据 |
|---|---|---|
| 0.2 m/s 前进 3 秒仿真时间 | {checks['forward']['dx']:.3f} m，目标 0.6±0.15 m | evidence/motion.json |
| 0.2 m/s 横移 3 秒仿真时间 | {checks['lateral']['dy']:.3f} m，目标 0.6±0.15 m | evidence/motion.json |
| 0.3 rad/s 转向 2 秒 | {checks['yaw']['dyaw']:.3f} rad，目标 0.6±0.15 rad | evidence/motion.json |
| 约 90° 航向后前进 | 世界 Δx={checks['body_frame_at_90deg']['world_dx']:.4f} m，Δy={checks['body_frame_at_90deg']['world_dy']:.4f} m；机体 vx={checks['body_frame_at_90deg']['body_vx']:.3f} m/s | evidence/motion.json |
| 零指令停止 | {checks['zero_command_stop']['stop_wall_seconds']:.3f} 秒 | evidence/motion.json |
| 输入超时停止 | {checks['watchdog_stop']['stop_wall_seconds']:.3f} 秒，低于 1 秒要求 | evidence/motion.json |
| 限幅 | 平移向量模长 {checks['limits']['max_linear']:.3f} m/s；转向 {checks['limits']['max_angular']:.3f} rad/s | evidence/motion.json |
| 暂停超过超时阈值后恢复 | 场景实际位置变化 {checks['pause_resume_no_stale_command']['resume_displacement']:.9f} m；航向变化 {checks['pause_resume_no_stale_command']['resume_yaw_change']:.9f} rad | evidence/motion.json |
| 键盘 W/S/A/D/Q/E | 六个键实际机体系速度逐项通过 | evidence/motion.json |
| 空格 / Esc | 分别 {checks['keyboard_space_stop']['stop_wall_seconds']:.3f} / {checks['keyboard_escape_stop']['stop_wall_seconds']:.3f} 秒内停止 | evidence/motion.json |
| Ctrl+C、终端断开、重复键盘实例 | 停止且无遗留进程/发布者；第二实例被拒绝 | evidence/keyboard-lifecycle.json |
| Gazebo 原始反馈与定位 | {raw['matched_samples']} 对同时间戳消息，位置/速度最大差 {raw['max_difference']} | evidence/motion.json |
| 反馈频率 | {checks['feedback_50hz']['hz_sim_time']:.2f} Hz（仿真时间） | evidence/motion.json |
| TF | map→odom→base_link 连通，TF/定位位置误差 {checks['tf_connected']['pose_error']} | evidence/motion.json |
| 模型 / 发布者 | 一个 sentry；clock、原始状态、定位、动态/静态 TF 各一个发布者；无 fixture | evidence/model-count.json、motion.json |
| 停止 / 重启 | 原进程全部退出，话题发布者归零，再启动恢复单一模型和发布者 | evidence/first-shutdown.json、shutdown-publishers.json、final-restart.json |
| 重复启动场景 | 锁拦截第二实例 | evidence/duplicate-launch.json |
| 生产启动约定 | 3 项既有回归通过 | evidence/bringup-regression.txt |

停止判据：线速度低于 0.02 m/s、角速度低于 0.05 rad/s。暂停测试在服务实际暂停前持续发运动指令，并读取**暂停期间持续更新的 Gazebo scene pose**；不使用命令积分或暂停前最后一帧 odometry 代替实际状态。

## 图形与性能

- 先运行 Fortress 自带 shapes 场景检查图形，再运行本仓库场景。
- 默认 VMware SVGA3D / OpenGL 4.3 路径持续白屏，报告 Accelerated=no；失败截图和 Ogre 日志保留。
- 随后使用 **llvmpipe（CPU 软件渲染）、OpenGL 4.5、Ogre2、XWayland** 成功显示。启动脚本对本阶段默认设置 LIBGL_ALWAYS_SOFTWARE=1，不修改用户全局桌面。
- 实际完成滚轮缩放、中键环绕；模型未丢失。截图见下方。
- 最终版本连续监测 **{st['duration_wall']:.1f} 秒**，所有采样均保持同一组 6 个仿真进程存活。
- 平均实时率 **{st['rtf_mean']:.3f}**，5 秒窗口最低 **{st['rtf_min']:.3f}**。
- 仿真进程 CPU 均值 **{st['cpu_mean_one_core']:.1f}%**（100%=一个逻辑核，4 vCPU 总预算约 400%）。
- 仿真进程 RSS 合计峰值 **{st['rss_peak_mib']:.1f} MiB**。RSS 会重复计入共享页，未将它当成独占物理内存。
- 监测包含 Gazebo GUI、server、bridge、adapter 和 launcher；原始 5 秒采样保存在 evidence/stability.json。实时率约 1 表示仿真时间接近真实时间；这不是激光/导航重负载的性能承诺。

![实际机器人场景](evidence/sentry-initial.png)

![实际缩放后的场景](evidence/sentry-zoom.png)

![实际旋转视角后的场景](evidence/sentry-orbit.png)

## 实现边界和已处理问题

模型名为“基础导航仿真模型”，采用 VelocityControl、OdometryPublisher，无碰撞几何、轮胎、悬架、坡道动力学、雷达或 IMU。
未启动 HDL、点云、规划器、MPC、生产硬件，也未进行自动导航、碰撞或避障验收。
接口、命令、ROS domain 26、时间和第二阶段接入说明详见 [README](README.md)。

已修复两个现场发现的问题：
1. 终端断开时键盘程序可能循环发送零指令，测试失败清理也可能遗留子进程。现改为 EOF 退出、重复实例锁、退出发送零速度，验收使用独立进程组清理；生命周期回归通过。
2. Fortress VelocityControl 在暂停中缓存旧速度，恢复首步曾产生 2 mm 位移。新增仅负责恢复零速度的 ResumeGuard；两步内清除缓存影响，不修改位姿；最终原始 scene pose 验证位移及转角均为零。

保留的兼容限制：
- 默认 SVGA3D 白屏原因未在系统层根治，使用已验收的软件渲染路径。
- 独立 shapes 测试程序关闭时出现过 Qt/QML 析构崩溃；正式 ROS launch 场景的停止和重启已验证，不存在遗留发布者。相关历史日志在 graphics-software.log。
- Ogre2 软件路径会将不支持的 8 倍抗锯齿降为 0，Qt 统计布局会输出警告；实际三维显示、交互和持续运行均已验证。

## 环境与磁盘

- 新增 {disk['packages_added']} 个包，无已有包版本变更，未删除原有包。
- 主要依赖 ros-humble-ros-gz、Gazebo Fortress 6.18.0、mesa-utils；xdotool 仅供图形验收。
- 新增包 Installed-Size 合计约 **{sizes['added_installed_mib']:.1f} MiB**；包精确清单见 evidence/packages-added.json。
- 测量时可用空间 **{disk['available_bytes']/2**30:.2f} GiB**。
- 相对安装前已用空间增加 **{(disk['used_bytes']-42796503040)/2**30:.2f} GiB**，包含 apt 缓存、索引、构建和运行诊断等，不能等同于依赖包本体。
- 没有修改 VMware CPU/内存/磁盘配置，没有重装系统、创建完整备份、清理旧构建或 Docker 资源。
- 独立构建目录仅 build_sentry_gazebo、install_sentry_gazebo、log_sentry_gazebo，编译并发最多 2。

## 证据说明

最终结论只使用 motion.json、keyboard-lifecycle.json、stability.json 和正式重启证据。
文件名含 before、probe、diagnostic 的记录是排障历史，保留失败现场，不能拿它们当成最终通过结果。
所有交付源文件、中文说明、截图、测试脚本和证据均在 RM_Sentry_2026 仓库。
"""
(DOC/'ACCEPTANCE.md').write_text(report)
print('Wrote',DOC/'ACCEPTANCE.md')
