#!/usr/bin/env python3
"""Own simulation command selection, trajectory acceptance and latched autonomous safety."""
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from autonomy_geometry import StaticSafetyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from trajectory_generation.msg import TrajectoryPoly

# Return seconds without replacing the sensor's measurement timestamp.
def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

# Inspect analytic derivative extrema; malformed polynomials never become controller input.
def trajectory_limits(message):
    durations = np.asarray(message.duration, dtype=float)
    x = np.asarray(message.coef_x, dtype=float).reshape(-1, 4)
    y = np.asarray(message.coef_y, dtype=float).reshape(-1, 4)
    if not (len(durations) == len(x) == len(y) > 0):
        raise ValueError('coefficient lengths')
    if not np.isfinite(durations).all() or (durations <= 0).any() or durations.sum() > 300:
        raise ValueError('invalid durations')
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError('non-finite coefficients')
    speed = acceleration = 0.
    for index, (cx, cy, duration) in enumerate(zip(x, y, durations)):
        vx, vy = np.polyder(cx), np.polyder(cy)
        if index+1 < len(durations):
            if math.hypot(np.polyval(cx,duration)-x[index+1,3],np.polyval(cy,duration)-y[index+1,3])>1e-4:
                raise ValueError('discontinuous position')
            if math.hypot(np.polyval(vx,duration)-x[index+1,2],np.polyval(vy,duration)-y[index+1,2])>1e-4:
                raise ValueError('discontinuous velocity')
        slope = np.polyadd(np.polymul(vx, np.polyder(vx)), np.polymul(vy, np.polyder(vy)))
        extrema = [0., duration] + [float(t.real) for t in np.roots(slope)
                                  if abs(t.imag) < 1e-8 and 0 < t.real < duration]
        speed = max(speed, *(math.hypot(np.polyval(vx, t), np.polyval(vy, t)) for t in extrema))
        acceleration = max(acceleration, *(math.hypot(np.polyval(np.polyder(vx), t),
                            np.polyval(np.polyder(vy), t)) for t in [0., duration]))
    return float(durations.sum()), speed, acceleration, (
        float(np.polyval(x[-1], durations[-1])), float(np.polyval(y[-1], durations[-1])))

class AutonomyGuard(Node):
    # Wire a single protected output; all fail states require a fresh goal or manual takeover.
    def __init__(self):
        super().__init__('sentry_autonomy_guard')
        self.static_grid = StaticSafetyGrid.load()
        self.prediction_stamp = -math.inf
        self.mode = 'auto'
        self.state = 'idle'
        self.reason = 'waiting for goal'
        self.goal = None
        self.endpoint = None
        self.odom = None
        self.odom_received = self.cloud_received = self.command_received = -math.inf
        self.cloud_stamp = -math.inf
        self.command = Twist()
        self.sim_previous = None
        self.clock_progress = time.monotonic()
        self.accepted_at = -math.inf
        self.valid_until = -math.inf
        self.requested_at = -math.inf
        self.goal_wall = -math.inf
        self.dwell = None
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/sim/planner_goal', 10)
        self.trajectory_pub = self.create_publisher(TrajectoryPoly, '/global_trajectory', 10)
        self.status_pub = self.create_publisher(String, '/sim/autonomy/status', 10)
        self.arrival_pub = self.create_publisher(Bool, '/sim/arrived', 10)
        self.create_subscription(PoseStamped, '/goal', self.on_goal, 10)
        self.create_subscription(TrajectoryPoly, '/sim/planned_trajectory', self.on_trajectory, 10)
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/aligned_points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, '/sim/auto_cmd_vel', self.on_auto, 1)
        self.create_subscription(PathMsg, '/tracking/mpc_predicted_path', self.on_prediction, 1)
        self.create_subscription(Twist, '/sim/manual_cmd_vel', self.on_manual, 1)
        self.create_subscription(String, '/sim/control_mode', self.on_mode, 10)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_status = 0.

    # Clear the active controller reference as well as the output; never silently resume old work.
    def invalidate(self, reason, state='stopped'):
        self.state, self.reason = state, reason
        self.command = Twist()
        self.command_received = -math.inf
        self.endpoint = None
        self.dwell = None
        self.trajectory_pub.publish(TrajectoryPoly())
        self.command_pub.publish(Twist())

    # Store validated measured pose and body twist, refusing frame or numeric corruption.
    def on_odom(self, message):
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        vals = [p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z]
        if message.header.frame_id != 'map' or message.child_frame_id != 'base_link' or not all(map(math.isfinite, vals)):
            self.invalidate('invalid localization')
            return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .02:
            self.invalidate('invalid orientation')
            return
        self.odom, self.odom_received = message, time.monotonic()

    # Only successfully aligned measurement-time clouds satisfy the sensor watchdog.
    def on_cloud(self, message):
        if message.header.frame_id == 'map' and message.width * message.height > 0:
            self.cloud_received = time.monotonic()
            self.cloud_stamp = seconds(message.header.stamp)

    # Reject a stale localization/cloud even when a publisher keeps replaying its payload.
    def healthy(self, wall, sim):
        if self.odom is None or wall-self.odom_received > .5:
            return 'localization timeout'
        age = sim-seconds(self.odom.header.stamp)
        if age < -.05 or age > .5:
            return 'localization timestamp'
        if wall-self.cloud_received > .8 or not -.05 <= sim-self.cloud_stamp <= .8:
            return 'cloud or measurement TF timeout'
        return ''

    # New requests stop the previous trajectory before asking the planner; failure cannot drive old paths.
    def on_goal(self, message):
        self.invalidate('planning', 'planning')
        self.goal = None
        sim = self.get_clock().now().nanoseconds * 1e-9
        if self.mode != 'auto':
            self.invalidate('select auto before sending a new goal')
            return
        if message.header.frame_id != 'map' or not all(map(math.isfinite, [
                message.pose.position.x, message.pose.position.y])):
            self.invalidate('invalid goal frame or coordinates')
            return
        fault = self.healthy(time.monotonic(), sim)
        if fault:
            self.invalidate(fault)
            return
        self.goal = copy.deepcopy(message)
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.requested_at = sim
        self.goal_wall = time.monotonic()
        self.goal_pub.publish(self.goal)

    # Accept only current bounded references for this goal; no retiming that changes moving initial speed.
    def on_trajectory(self, message):
        if self.mode != 'auto' or self.goal is None or self.state not in ('planning', 'tracking'):
            return
        sim = self.get_clock().now().nanoseconds * 1e-9
        try:
            duration, speed, acceleration, endpoint = trajectory_limits(message)
            if seconds(message.start_time) < self.requested_at - .05:
                return
            if not -.05 <= sim-seconds(message.start_time) <= .5:
                raise ValueError('stale trajectory timestamp')
            if speed > .5 or acceleration > .4:
                raise ValueError('reference exceeds simulation speed/acceleration limits')
            if math.hypot(endpoint[0]-self.goal.pose.position.x, endpoint[1]-self.goal.pose.position.y) > .2:
                return
            if not self.static_grid.trajectory_free(message):
                raise ValueError('reference intersects occupied footprint grid')
            if self.healthy(time.monotonic(), sim):
                raise ValueError('stale observation')
        except (ValueError, IndexError, TypeError) as error:
            self.invalidate(str(error))
            return
        self.prediction_stamp = -math.inf
        self.endpoint = endpoint
        self.accepted_at, self.valid_until = sim, sim+duration+5.
        self.state, self.reason = 'tracking', 'valid trajectory'
        self.command_received = -math.inf
        self.trajectory_pub.publish(message)

    # Reject predicted footprint intrusion before allowing its associated motion command.
    def on_prediction(self, message):
        if self.state != 'tracking': return
        stamp = seconds(message.header.stamp)
        if stamp < self.accepted_at: return
        if not self.static_grid.prediction_free(message):
            self.invalidate('MPC prediction intersects occupied footprint grid')
            return
        self.prediction_stamp = stamp

    # Controller output is only stored for an accepted trajectory, not used to arm the guard.
    def on_auto(self, message):
        if self.mode == 'auto' and self.state == 'tracking':
            self.command = message
            self.command_received = time.monotonic()

    # Manual commands cannot race the controller: a mode change invalidates the autonomous reference.
    def on_manual(self, message):
        if self.mode == 'manual':
            self.command = message
            self.command_received = time.monotonic()

    # Explicit mode selection always clears old motion; returning to auto requires a new goal.
    def on_mode(self, message):
        if message.data in ('auto','manual','stop'):
            self.mode = message.data
            self.goal = None
            self.invalidate('mode changed', 'manual' if self.mode == 'manual' else 'idle')

    # Use wall time for fail-safe operation during pause; successful arrival also requires low measured speed.
    def tick(self):
        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds * 1e-9
        if self.sim_previous is None or sim != self.sim_previous:
            if self.sim_previous is not None and sim < self.sim_previous:
                self.goal = None
                self.invalidate('simulation time reset')
            self.clock_progress = wall
        self.sim_previous = sim
        fault = self.healthy(wall, sim)
        if wall-self.clock_progress > .5:
            fault = 'simulation paused'
        if fault and self.mode == 'manual':
            self.command = Twist()
            self.command_received = -math.inf
        if self.state == 'planning' and fault:
            self.invalidate(fault)
        if self.state == 'planning' and wall-self.goal_wall > 8:
            self.invalidate('planner returned no valid path')
        if self.state == 'tracking':
            if fault:
                self.invalidate(fault)
            elif sim > self.valid_until:
                self.invalidate('trajectory expired')
            elif self.endpoint is not None:
                p, v = self.odom.pose.pose.position, self.odom.twist.twist
                distance = math.hypot(p.x-self.endpoint[0], p.y-self.endpoint[1])
                if distance <= .15 and math.hypot(v.linear.x,v.linear.y) <= .03 and abs(v.angular.z) <= .05:
                    if self.dwell is None: self.dwell = wall
                    if wall-self.dwell >= .5:
                        self.invalidate('position and measured speed settled for 0.5 s', 'arrived')
                else:
                    self.dwell = None
        output = Twist()
        if (self.state == 'tracking' or self.mode == 'manual') and not fault:
            if wall-self.command_received <= .5:
                values = [self.command.linear.x,self.command.linear.y,self.command.angular.z]
                if all(map(math.isfinite, values)):
                    moving = math.hypot(self.command.linear.x,self.command.linear.y)>1e-6
                    if self.mode == 'manual' or not moving or -.05 <= sim-self.prediction_stamp <= .5:
                        output = self.command
                    elif sim-self.accepted_at > .5:
                        self.invalidate('MPC prediction timeout')
                elif self.state == 'tracking':
                    self.invalidate('non-finite control')
            elif self.state == 'tracking' and sim-self.accepted_at > .5:
                self.invalidate('controller timeout')
        self.command_pub.publish(output)
        self.arrival_pub.publish(Bool(data=self.state == 'arrived'))
        if wall-self.last_status > .2:
            self.status_pub.publish(String(data=json.dumps({'mode':self.mode,'state':self.state,'reason':self.reason})))
            self.last_status = wall

# Shutdown still sends zero; downstream wall-time protection covers a killed guard.
def main():
    rclpy.init()
    node = AutonomyGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok(): node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
