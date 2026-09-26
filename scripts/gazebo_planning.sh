#!/usr/bin/env bash
# Build and run isolated stage-two simulation; share the stage-one safety lock.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}" ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="sentry_basic_${USER}_${ROS_DOMAIN_ID}"
export QT_QPA_PLATFORM=xcb PYTHONNOUSERSITE=1
MODE="${1:-start}"
if [[ "$MODE" == build ]]; then
  cd "$WS"
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  colcon --log-base log_sentry_sim2 build --build-base build_sentry_sim2 --install-base install_sentry_sim2 \
    --base-paths src/sentry_gazebo src/sentry_bringup src/trajectory_generation src/waypoint_generator \
    --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
  exit
fi
if [[ ! -f "$WS/install_sentry_sim2/local_setup.bash" ]]; then
  bash "$ROOT/scripts/gazebo_planning.sh" build
fi
source "$WS/install_sentry_sim2/local_setup.bash"
if [[ "$MODE" == keyboard ]]; then
  exec ros2 run sentry_gazebo keyboard.py --ros-args -p use_sim_time:=true
fi
[[ "$MODE" == probe || "$MODE" == start || "$MODE" == map ]] || { echo "Usage: $0 [start|probe|map|keyboard|build]"; exit 2; }
[[ -n "${DISPLAY:-}" ]] || { echo '请在 Ubuntu 桌面终端启动。'; exit 1; }
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sentry-gazebo-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo '已有仿真运行，请先在原终端 Ctrl+C 停止。'; exit 1; }
python3 "$ROOT/ros2_ws/src/sentry_gazebo/scripts/preflight.py"
export LIBGL_ALWAYS_SOFTWARE=1
if [[ "$MODE" == probe ]]; then
  exec ros2 launch sentry_gazebo lidar_probe.launch.py
elif [[ "$MODE" == map ]]; then
  exec ros2 launch sentry_bringup gazebo_planning.launch.py enable_planner:=false
fi
exec ros2 launch sentry_bringup gazebo_planning.launch.py
