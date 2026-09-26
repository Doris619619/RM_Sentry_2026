<!-- 文件用途：解释第二阶段机器记录、截图、日志和录像的证据来源及用途。 -->

# 证据索引

JSON 为测试或采样脚本生成的原始结果，不插入注释或修改数值。日志保留异常和历史失败。
本文为这些不可注释数据文件提供用途说明。

- probe.json / m0-performance.json / m0-lidar.png：小场景真实 gpu_lidar 的测距、移动变化及 GUI 性能。
- map-generation.json / map-validation.json：源地图哈希、生成几何覆盖、地标和真实地图/点云。
- planning-cases.json：五组各三次的真实仿真起终点、完整多项式系数和时长、采样结果；9 条失败不可删除。
- planning-cases-initial-diagnostic.*：初次发现跳变时的一条记录，属诊断历史。
- planning-performance.json：修正材质与 Marker 后 310 秒双 GUI 性能，含每 5 秒进程树样本。
- performance-before-rviz-fix.*：显示配置修正前的历史测量，不能替代最终 GUI 验收。
- performance-goal-display.*：增加持久目标箭头后的第二次 310 秒性能测量。
- cloud-negative.*：domain 27 中明确合成的负例，只验证适配器，不冒充传感器。
- planner-negative-*.log：domain 28 的既有规划器负例；与实际运行隔离。
- corrected-goal.json：真实占据目标依照原邻点规则修正后的端点验证。
- source-audit.json / shutdown-handler.json：静态语法、地图和算法未改动检查；新节点 SIGTERM 正常退出验证。
- unreachable-goal.json：真实规划场景中不可修正占据目标的拒绝结果。
- gui-goal.json / goal-display.json：实际鼠标点击结果、RViz 目标显示订阅检查。
- pause-resume.json / lifecycle-stop.json / lifecycle-restart.json / first-stop.json：暂停、过期指令、进程与发布者退出、重启实体数量。
- stage1-regression/：第一阶段原测试逻辑复验结果，改用本目录输出以保留第一阶段原证据。
- regression.log / launch-contract.log：既有单元与启动测试；完整生产启动存在缺少 decision_node 的环境阻塞。
- build-*.log：本阶段构建结果；qa-window-tools-install.log：用于摆放窗口的 wmctrl 安装记录。
- stage2-planning.png：真实双 GUI 截图；demo-raw.webm 为真实双窗口原始录制。
- demo-events.json / demo-keyboard.txt / demo-*.log：键盘 PTY、鼠标目标与实际消息时间记录。
- planning-final.log：性能、目标及停止阶段完整日志，包含 RViz 退出 -6 异常。
- restart-first.log：一次重启运行及停止的日志；final-runtime.log 是交付时运行日志快照；goal-display-stop.log 是最终 GUI 性能测试后的退出记录。

录像中的手动控制通过真正的 keyboard.py 终端输入，目标通过 RViz 工具鼠标点击；
没有播放预制轨迹或生成伪点云。字幕与视频编码不改变仿真数据。
