#!/usr/bin/env bash
# Build and run isolated autonomous simulation; share the stage-one safety lock.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}" ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="sentry_basic_${USER}_${ROS_DOMAIN_ID}"
export QT_QPA_PLATFORM=xcb PYTHONNOUSERSITE=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0 QT_SCALE_FACTOR=1 QT_ENABLE_HIGHDPI_SCALING=0
DEPENDENCY_SETUP="${SENTRY_DEPENDENCY_SETUP:-$WS/install_closure_fix/local_setup.bash}"
if [[ ! -f "$DEPENDENCY_SETUP" ]]; then
  echo '缺少 OCS2 依赖环境；按 ros2_ws/README.md 构建后，将 SENTRY_DEPENDENCY_SETUP 指向 install/local_setup.bash。'
  exit 1
fi
source "$DEPENDENCY_SETUP"
MODE="${1:-start}"
if [[ "$MODE" == build ]]; then
  cd "$WS"
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  colcon --log-base log_sentry_autonomy build --build-base build_sentry_autonomy --install-base install_sentry_autonomy \
    --base-paths src/sentry_gazebo src/sentry_bringup src/trajectory_generation src/waypoint_generator src/trajectory_tracking \
    --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
  exit
fi
if [[ ! -f "$WS/install_sentry_autonomy/local_setup.bash" ]]; then
  bash "$ROOT/scripts/gazebo_autonomy.sh" build
fi
source "$WS/install_sentry_autonomy/local_setup.bash"
if [[ "$MODE" == keyboard ]]; then
  ros2 topic pub --once /sim/control_mode std_msgs/msg/String '{data: manual}'
  exec ros2 run sentry_gazebo keyboard.py --ros-args -p use_sim_time:=true -r /cmd_vel:=/sim/manual_cmd_vel
fi
if [[ "$MODE" == auto || "$MODE" == stop ]]; then
  exec ros2 topic pub --once /sim/control_mode std_msgs/msg/String "{data: $MODE}"
fi
if [[ "$MODE" == restart ]]; then
  python3 "$ROOT/ros2_ws/src/sentry_gazebo/scripts/owned_launch.py" stop
  MODE=start
fi
[[ "$MODE" == start ]] || { echo "Usage: $0 [start|keyboard|auto|stop|restart|build]"; exit 2; }
[[ -n "${DISPLAY:-}" ]] || { echo '请在 Ubuntu 桌面终端启动。'; exit 1; }
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sentry-gazebo-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo '已有仿真运行，请先在原终端 Ctrl+C 停止。'; exit 1; }
python3 "$ROOT/ros2_ws/src/sentry_gazebo/scripts/preflight.py"
export LIBGL_ALWAYS_SOFTWARE=1
python3 "$ROOT/ros2_ws/src/sentry_gazebo/scripts/owned_launch.py" register
exec ros2 launch sentry_gazebo autonomy.launch.py localization:="${SENTRY_LOCALIZATION:-truth}"
