#!/usr/bin/env bash
# Convenience entry point for the complete ROS 2 simulation, from any directory.
set -eo pipefail

SIM_ROOT="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
SIM_ACTION="${1:-start}"

usage() {
  cat <<'EOF'
用法：sim.sh [命令]（不加命令即启动仿真）

  start      启动默认 HIT 开源 RMUC2024 赛场仿真，初始静止
  menu       打开交互控制菜单（在另一个终端运行）
  patrol     自动模式 + 巡逻场景
  supply     自动模式 + 低血量返补给场景
  stop       停止运动并结束模拟比赛
  auto       只切换自动模式
  keyboard   手动接管，按终端提示控制
  status     查看 ROS 节点和 MCU 状态
  restart    完整重启仿真，恢复初始静止状态
  quit       完整退出已登记的仿真
  build      重新构建仿真包
  help       显示帮助

自动加载 ROS 2 和本项目的安装环境，无需 cd 或手动 source。
可通过 ROS_DOMAIN_ID 和 SENTRY_DEPENDENCY_SETUP 覆盖默认设置。
EOF
}

if (( $# > 1 )); then usage >&2; exit 2; fi
case "$SIM_ACTION" in
  help|-h|--help) usage; exit 0 ;;
  start|menu|patrol|supply|stop|auto|keyboard|manual|status|restart|quit|build) ;;
  *) printf '未知命令：%s\n' "$SIM_ACTION" >&2; usage >&2; exit 2 ;;
esac

# Resolve an explicit relative setup path before leaving the caller's directory.
if [[ -n "${SENTRY_DEPENDENCY_SETUP:-}" ]]; then
  export SENTRY_DEPENDENCY_SETUP="$(realpath -m "$SENTRY_DEPENDENCY_SETUP")"
else
  for sim_prefix in install_sim_deps install_closure_fix install; do
    if [[ -f "$SIM_ROOT/ros2_ws/$sim_prefix/local_setup.bash" ]]; then
      export SENTRY_DEPENDENCY_SETUP="$SIM_ROOT/ros2_ws/$sim_prefix/local_setup.bash"
      break
    fi
  done
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "${SENTRY_DEPENDENCY_SETUP:-}" ]]; then
  echo '缺少 ROS 2 Humble 或依赖安装环境，请按 docs/simulation/ROS2仿真启动指南.md 准备。' >&2
  exit 1
fi
cd "$SIM_ROOT"
# Keep camera SDK / Conda libraries out of this process's ROS/PCL runtime.
# In particular, the MVS SDK ships a libusb that lacks symbols required by PCL.
sim_library_paths=()
IFS=: read -r -a sim_inherited_paths <<< "${LD_LIBRARY_PATH:-}"
for sim_library_path in "${sim_inherited_paths[@]}"; do
  case "$sim_library_path" in
    ''|/opt/MVS/*|*/miniconda*/*|*/anaconda*/*) continue ;;
    *) sim_library_paths+=("$sim_library_path") ;;
  esac
done
export LD_LIBRARY_PATH="$(IFS=:; echo "${sim_library_paths[*]}")"
source /opt/ros/humble/setup.bash
source "$SENTRY_DEPENDENCY_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}" ROS_LOCALHOST_ONLY=1
export PYTHONNOUSERSITE=1

if [[ "$SIM_ACTION" == build ]]; then
  # Build the runtime dependency closure too: the full launch requires HDL/NDT.
  (
    cd "$SIM_ROOT/ros2_ws"
    export CMAKE_BUILD_PARALLEL_LEVEL=2
    export CMAKE_PREFIX_PATH="$PWD/.deps/livox-sdk2:${CMAKE_PREFIX_PATH:-}"
    colcon build --build-base build_sim_deps --install-base install_sim_deps \
      --parallel-workers 2 --packages-up-to trajectory_tracking hdl_localization \
      --cmake-args -DBUILD_TESTING=OFF \
      -DOpenCV_DIR="/usr/lib/$(gcc -dumpmachine)/cmake/opencv4"
  )
  export SENTRY_DEPENDENCY_SETUP="$SIM_ROOT/ros2_ws/install_sim_deps/local_setup.bash"
  exec bash "$SIM_ROOT/scripts/gazebo_full.sh" build
fi
if [[ ! -f "$SIM_ROOT/ros2_ws/install_sentry_autonomy/local_setup.bash" ]]; then
  printf '尚未构建仿真包，请先运行：\n  %s build\n' "$SIM_ROOT/sim.sh" >&2
  exit 1
fi
source "$SIM_ROOT/ros2_ws/install_sentry_autonomy/local_setup.bash"

publish() {
  # A missing subscriber should produce a useful error, not wait indefinitely.
  if timeout 10s ros2 topic pub --once -w 1 "$1" std_msgs/msg/String "{data: $2}"; then
    return 0
  else
    echo '消息发送失败或等待订阅者超时。请确认仿真已启动，且两个终端的 ROS_DOMAIN_ID 相同。' >&2
    return 1
  fi
}

case "$SIM_ACTION" in
  start|restart)
    if ! ros2 pkg prefix hdl_localization >/dev/null 2>&1; then
      printf '缺少 HDL/NDT 定位运行依赖，请先执行：\n  %s build\n' "$SIM_ROOT/sim.sh" >&2
      exit 1
    fi
    printf '启动 HIT 开源 RMUC2024 赛场仿真（ROS domain %s）。另开终端运行：\n  %s menu\n启动终端按 Ctrl+C 完整退出。\n' "$ROS_DOMAIN_ID" "$SIM_ROOT/sim.sh"
    exec bash "$SIM_ROOT/scripts/gazebo_full.sh" "$SIM_ACTION"
    ;;
  patrol|supply)
    publish /sim/referee/scenario stop
    publish /sim/control_mode auto
    publish /sim/referee/scenario "$SIM_ACTION"
    ;;
  stop)
    publish /sim/control_mode stop
    publish /sim/referee/scenario stop
    ;;
  auto) publish /sim/control_mode auto ;;
  keyboard|manual) exec bash "$SIM_ROOT/scripts/gazebo_full.sh" keyboard ;;
  quit) exec python3 "$SIM_ROOT/ros2_ws/src/sentry_gazebo/scripts/owned_launch.py" stop ;;
  status)
    ros2 node list
    echo 'MCU 状态：'
    timeout 10s ros2 topic echo --once /sim/mcu/status || {
      echo '未收到 MCU 状态，请检查仿真是否已启动。' >&2
      exit 1
    }
    ;;
  menu)
    while true; do
      printf '\n1 巡逻  2 返补给  3 停止运动  4 手动接管  5 查看状态  6 完整退出仿真  0 关闭菜单\n'
      read -r -p '选择操作：' sim_choice || break
      case "$sim_choice" in
        1) sim_command=patrol ;; 2) sim_command=supply ;; 3) sim_command=stop ;;
        4) sim_command=keyboard ;; 5) sim_command=status ;; 6) sim_command=quit ;;
        0) break ;; *) echo '请输入 0–6。'; continue ;;
      esac
      # A failed operation returns to the menu so the user can retry.
      bash "$SIM_ROOT/sim.sh" "$sim_command" || echo '操作未完成，可检查启动终端日志后重试。' >&2
    done
    ;;
esac
