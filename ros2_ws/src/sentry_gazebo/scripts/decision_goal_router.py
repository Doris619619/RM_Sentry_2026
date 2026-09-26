#!/usr/bin/env python3
"""Route decision goals through the same protected goal input, deduplicating behavior-tree ticks."""
import copy,math,time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped,PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt8
from rclpy.qos import qos_profile_sensor_data

class DecisionGoalRouter(Node):
    # Wait for estimated pose and aligned sensor data; no fixture or truth input can bypass this gate.
    def __init__(self):
        super().__init__('sentry_decision_goal_router')
        self.goal=None;self.forwarded=None;self.progress=0;self.started=None
        self.odom_wall=self.cloud_wall=-math.inf
        self.pub=self.create_publisher(PoseStamped,'/goal',10)
        self.create_subscription(PointStamped,'/sim/decision_goal',self.on_goal,10)
        self.create_subscription(UInt8,'/referee/game_progress',self.on_referee,10)
        self.create_subscription(Odometry,'/localization/odometry',lambda m:setattr(self,'odom_wall',time.monotonic()),10)
        self.create_subscription(PointCloud2,'/aligned_points',lambda m:setattr(self,'cloud_wall',time.monotonic()),qos_profile_sensor_data)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    # Ignore malformed decision requests before they reach the autonomous supervisor.
    def on_goal(self,msg):
        if msg.header.frame_id=='map' and all(math.isfinite(v) for v in (msg.point.x,msg.point.y)):
            self.goal=msg

    # A new game episode may repeat the same legitimate goal; an ended game clears only the router cache.
    def on_referee(self,msg):
        if msg.data!=self.progress:
            self.forwarded=None;self.started=time.monotonic() if msg.data==4 else None
        self.progress=msg.data

    # Continuous behavior-tree publication cannot repeatedly reset the planner's trajectory clock.
    def tick(self):
        now=time.monotonic()
        if self.progress!=4 or self.started is None or now-self.started<1. or self.goal is None:return
        if now-self.odom_wall>.5 or now-self.cloud_wall>.8:return
        key=(self.goal.point.x,self.goal.point.y)
        if key==self.forwarded:return
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x=key[0];msg.pose.position.y=key[1];msg.pose.orientation.w=1.
        self.pub.publish(msg);self.forwarded=key

def main():
    """End the router with the owning launch; it never publishes a velocity command."""
    rclpy.init();node=DecisionGoalRouter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
