#!/usr/bin/env bash
# Launch planning-side semi-sim (RViz + global searcher) inside the X11-enabled Docker container.
# Usage:
#   ./run_planning_sim.sh              # global planner + RViz
#   ./run_planning_sim.sh tracking     # local tracking node (run in a 2nd terminal)
#   ./run_planning_sim.sh both         # both in one container (two processes)
#   ./run_planning_sim.sh rviz         # restart RViz only
#   ./run_planning_sim.sh stop         # stop all planning sim processes

set -euo pipefail

CONTAINER="${CONTAINER:-rm_sentry_gui}"
IMAGE="${IMAGE:-rm_sentry_noetic:built}"
REPO="/home/liangys/RM_Sentry_2026"
DISPLAY_NUM="${DISPLAY_NUM:-:0}"
MODE="${1:-global}"

ensure_xhost() {
  DISPLAY="$DISPLAY_NUM" xhost +local: >/dev/null 2>&1 || true
  DISPLAY="$DISPLAY_NUM" xhost +local:docker >/dev/null 2>&1 || true
}

ensure_container() {
  if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "[run] creating $CONTAINER with X11..."
    docker run -d --name "$CONTAINER" \
      --network host \
      -e DISPLAY="$DISPLAY_NUM" \
      -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "$REPO:/ws/RM_Sentry_2026" \
      "$IMAGE" \
      sleep infinity
  elif [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" != "true" ]]; then
    echo "[run] starting $CONTAINER..."
    docker start "$CONTAINER" >/dev/null
  fi
}

ros_env='
export DISPLAY='"$DISPLAY_NUM"'
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/noetic/setup.bash
source /ws/RM_Sentry_2026/HIT_code/sentry_planning_ws/devel/setup.bash
'

ensure_xhost
ensure_container

case "$MODE" in
  global)
    echo "[run] launching global_searcher_sim (RViz + trajectory_generation)..."
    echo "[hint] Use tool '3D Nav Goal' (Goal3DTool), click WHITE free space."
    echo "[hint] Do NOT close the RViz window with X; if it dies it should respawn."
    echo "[hint] If window vanishes:  $0 rviz"
    docker exec -it \
      -e DISPLAY="$DISPLAY_NUM" \
      -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      -e XDG_RUNTIME_DIR=/tmp/runtime-root \
      "$CONTAINER" bash -lc "$ros_env
mkdir -p /tmp/runtime-root
roslaunch trajectory_generation global_searcher_sim.launch
"
    ;;
  rviz)
    echo "[run] restarting RViz only (planning nodes keep running)..."
    docker exec -d \
      -e DISPLAY="$DISPLAY_NUM" \
      -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      -e XDG_RUNTIME_DIR=/tmp/runtime-root \
      "$CONTAINER" bash -lc "$ros_env
mkdir -p /tmp/runtime-root
pkill -f '/opt/ros/noetic/lib/rviz/rviz' 2>/dev/null || true
sleep 1
rosrun rviz rviz -d \$(rospack find trajectory_generation)/cfg/2D_map.rviz
"
    echo "[run] RViz restart requested. Check your desktop."
    ;;
  stop)
    echo "[run] stopping planning sim..."
    docker exec "$CONTAINER" bash -lc '
pkill -f "roslaunch trajectory_generation" 2>/dev/null || true
pkill -f "/opt/ros/noetic/lib/rviz/rviz" 2>/dev/null || true
pkill -f trajectory_generation 2>/dev/null || true
pkill -f waypoint_generator 2>/dev/null || true
' || true
    echo "[run] stopped."
    ;;
  tracking)
    echo "[run] launching trajectory_planning_sim (tracking_node)..."
    docker exec -it \
      -e DISPLAY="$DISPLAY_NUM" \
      -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      "$CONTAINER" bash -lc "$ros_env
roslaunch tracking_node trajectory_planning_sim.launch
"
    ;;
  both)
    echo "[run] launching global + tracking..."
    docker exec -it \
      -e DISPLAY="$DISPLAY_NUM" \
      -e QT_X11_NO_MITSHM=1 \
      -e LIBGL_ALWAYS_SOFTWARE=1 \
      "$CONTAINER" bash -lc "$ros_env
roslaunch trajectory_generation global_searcher_sim.launch &
sleep 3
roslaunch tracking_node trajectory_planning_sim.launch
"
    ;;
  test-x11)
    echo "[run] testing X11 with xeyes (2s)..."
    docker exec \
      -e DISPLAY="$DISPLAY_NUM" \
      "$CONTAINER" bash -lc "timeout 2 xeyes || true"
    echo "[run] X11 test done. If you saw eyes on screen, GUI works."
    ;;
  *)
    echo "Usage: $0 [global|tracking|both|rviz|stop|test-x11]"
    exit 1
    ;;
esac
