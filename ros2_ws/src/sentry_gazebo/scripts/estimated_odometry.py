#!/usr/bin/env python3
"""Adapt accepted HDL/NDT estimates to planar navigation; derive body twist only from estimated poses."""
import copy,json,math,time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from hdl_localization.msg import ScanMatchingStatus
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster

def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class EstimatedOdometry(Node):
    # No truth/MCU/command subscription exists here; matching confidence and estimated poses are the only inputs.
    def __init__(self):
        super().__init__('sentry_estimated_odometry')
        self.pending={};self.qualities={};self.previous=None;self.velocity=[0.,0.,0.]
        self.pose_history=deque(maxlen=4)
        self.pub=self.create_publisher(Odometry,'/localization/odometry',10)
        self.status=self.create_publisher(String,'/sim/localization/status',10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        identity=TransformStamped();identity.header.frame_id='map';identity.child_frame_id='odom'
        identity.transform.rotation.w=1.;self.static.sendTransform(identity)
        self.create_subscription(Odometry,'/sim/ndt/odometry',self.on_pose,10)
        self.create_subscription(ScanMatchingStatus,'/sim/ndt/status',self.on_quality,10)

    # Match asynchronous output/status by the original sensor timestamp and bound pending memory.
    def on_pose(self,msg):
        key=seconds(msg.header.stamp);self.pending[key]=msg
        self.pending={k:v for k,v in self.pending.items() if abs(key-k)<=1.}
        self.process(key)

    # A replayed or poor registration never refreshes the public localization watchdog.
    def on_quality(self,msg):
        key=seconds(msg.header.stamp);self.qualities[key]=msg
        self.qualities={k:v for k,v in self.qualities.items() if abs(key-k)<=1.}
        self.process(key)

    # Preserve measured x/y/yaw and timestamp; constrain z/roll/pitch to this explicitly planar simulator.
    def process(self,key):
        if key not in self.pending or key not in self.qualities:return
        source=self.pending.pop(key);quality=self.qualities.pop(key)
        age=self.get_clock().now().nanoseconds*1e-9-key
        p,q=source.pose.pose.position,source.pose.pose.orientation
        angle=yaw(q)
        good=(source.header.frame_id=='map' and source.child_frame_id=='base_link' and
            all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,quality.matching_error,quality.inlier_fraction)) and
            abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<.02 and abs(p.z)<.25 and
            -.05<=age<=.5 and quality.inlier_fraction>=.7 and quality.matching_error<=.04)
        if not good:
            self.status.publish(String(data=json.dumps({'valid':False,'reason':'registration quality or timestamp','age':age})))
            return
        current=(key,p.x,p.y,angle)
        if self.previous is None or key<=self.previous[0] or key-self.previous[0]>.6:
            self.previous=current;self.pose_history.clear();self.pose_history.append(current);self.velocity=[0.,0.,0.]
            return
        dt=key-self.previous[0]
        if math.hypot(p.x-self.previous[1],p.y-self.previous[2])/dt>1.5:
            self.previous=current;self.pose_history.clear();self.velocity=[0.,0.,0.]
            self.status.publish(String(data=json.dumps({'valid':False,'reason':'estimated pose discontinuity'})))
            return
        self.pose_history.append(current)
        oldest=self.pose_history[0];span=key-oldest[0]
        if span<=0:return
        vx=(p.x-oldest[1])/span;vy=(p.y-oldest[2])/span
        wz=math.atan2(math.sin(angle-oldest[3]),math.cos(angle-oldest[3]))/span
        self.velocity=[.5*old+.5*new for old,new in zip(self.velocity,(vx,vy,wz))]
        self.previous=current
        output=Odometry();output.header=copy.deepcopy(source.header);output.child_frame_id='base_link'
        output.pose.pose.position.x=p.x;output.pose.pose.position.y=p.y
        output.pose.pose.orientation.z=math.sin(angle/2);output.pose.pose.orientation.w=math.cos(angle/2)
        c,s=math.cos(angle),math.sin(angle);vx,vy,wz=self.velocity
        output.twist.twist.linear.x=c*vx+s*vy;output.twist.twist.linear.y=-s*vx+c*vy
        output.twist.twist.angular.z=wz
        # Nominal velocity uncertainty for differentiated scans, not a measured estimator covariance.
        output.twist.covariance[0]=output.twist.covariance[7]=.05**2
        output.twist.covariance[35]=.03**2
        self.pub.publish(output)
        tf=TransformStamped();tf.header=copy.deepcopy(output.header);tf.header.frame_id='odom';tf.child_frame_id='base_link'
        tf.transform.translation.x=p.x;tf.transform.translation.y=p.y;tf.transform.rotation=output.pose.pose.orientation
        self.tf.sendTransform(tf)
        self.status.publish(String(data=json.dumps({'valid':True,'source':'HDL/NDT estimated poses only',
            'matching_error_squared':quality.matching_error,'inlier_fraction':quality.inlier_fraction,
            'raw_z':p.z,'planar_projection':True})))

def main():
    """Leave downstream wall-time watchdogs responsible for process failure."""
    rclpy.init();node=EstimatedOdometry()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
