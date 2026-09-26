#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="sentry_basic_${USER}_${ROS_DOMAIN_ID}"
export QT_QPA_PLATFORM=xcb
export PYTHONNOUSERSITE=1
MODE="${1:-start}"
if [[ "$MODE" == build ]]; then
  cd "$WS"
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  colcon --log-base log_sentry_gazebo build --build-base build_sentry_gazebo \
    --install-base install_sentry_gazebo --base-paths src/sentry_gazebo src/sentry_bringup --packages-select sentry_gazebo sentry_bringup \
    --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
  exit
fi
if [[ ! -f "$WS/install_sentry_gazebo/local_setup.bash" ]]; then
  bash "$ROOT/scripts/gazebo_basic.sh" build
fi
source "$WS/install_sentry_gazebo/local_setup.bash"
if [[ "$MODE" == keyboard ]]; then
  exec ros2 run sentry_gazebo keyboard.py --ros-args -p use_sim_time:=true
elif [[ "$MODE" != start ]]; then
  echo "Usage: $0 [start|keyboard|build]" >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "请在已登录的 Ubuntu 桌面终端运行；当前没有 DISPLAY。" >&2
  exit 1
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sentry-gazebo-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo "此 ROS domain 的仿真已在运行。请先在原终端 Ctrl+C 停止。"; exit 1; }
python3 "$ROOT/ros2_ws/src/sentry_gazebo/scripts/preflight.py"
# This VMware SVGA3D path was tested and showed a blank Ogre2 viewport.
# Default to the verified llvmpipe path for this stage only; 0 retries hardware.
if [[ "${SENTRY_SOFTWARE_RENDERING:-1}" == 1 ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi
exec ros2 launch sentry_bringup gazebo_basic.launch.py
