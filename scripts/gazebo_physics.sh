#!/usr/bin/env bash
# Launch generic contact dynamics separately while preserving the shared simulation lock and ROS domain.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${SENTRY_DEPENDENCY_SETUP:-$ROOT/ros2_ws/install_closure_fix/local_setup.bash}"
source "$ROOT/ros2_ws/install_sentry_autonomy/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}" ROS_LOCALHOST_ONLY=1 IGN_PARTITION="sentry_basic_${USER}_${ROS_DOMAIN_ID}"
export QT_QPA_PLATFORM=xcb QT_AUTO_SCREEN_SCALE_FACTOR=0 QT_SCALE_FACTOR=1 QT_ENABLE_HIGHDPI_SCALING=0 LIBGL_ALWAYS_SOFTWARE=1
if [[ "${1:-start}" == keyboard ]]; then
  exec ros2 run sentry_gazebo keyboard.py --ros-args -p use_sim_time:=true
fi
[[ -n "${DISPLAY:-}" ]] || { echo '请在 Ubuntu 桌面终端启动。'; exit 1; }
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sentry-gazebo-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo '已有仿真运行，请先在原终端 Ctrl+C 停止。'; exit 1; }
python3 "$ROOT/ros2_ws/src/sentry_gazebo/scripts/preflight.py"
exec ros2 launch sentry_gazebo physics.launch.py
