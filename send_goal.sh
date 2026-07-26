#!/usr/bin/env bash
# Touchscreen-friendly: send a goal by command, no RViz mouse drag needed.
#
# Usage:
#   ./send_goal.sh                 # default test goal (2, 0)
#   ./send_goal.sh 0.0 0.0         # x y
#   ./send_goal.sh 2.0 -1.0 0.5    # x y z   (z must be >= 0)
#
# Then watch RViz map for path lines. You only need the touchscreen to look at RViz.

set -euo pipefail

CONTAINER="${CONTAINER:-rm_sentry_gui}"
X="${1:-2.0}"
Y="${2:-0.0}"
Z="${3:-0.5}"

echo "[goal] sending x=$X y=$Y z=$Z  ->  /goal"

docker exec "$CONTAINER" bash -lc "
source /opt/ros/noetic/setup.bash
source /ws/RM_Sentry_2026/HIT_code/sentry_planning_ws/devel/setup.bash
python3 - <<'PY'
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('goal_sender', anonymous=True)
pub = rospy.Publisher('/goal', PoseStamped, queue_size=1)
rospy.sleep(1.0)
m = PoseStamped()
m.header.frame_id = 'world'
m.header.stamp = rospy.Time.now()
m.pose.position.x = float('$X')
m.pose.position.y = float('$Y')
m.pose.position.z = float('$Z')
m.pose.orientation.w = 1.0
for _ in range(3):
    pub.publish(m)
    rospy.sleep(0.2)
print('goal published')
PY
"

echo "[goal] done. Look at RViz — path should appear on the map."
echo "[goal] try other free-space points if needed:"
echo "       ./send_goal.sh -2 2"
echo "       ./send_goal.sh 3 -2"
