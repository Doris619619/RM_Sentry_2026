#!/usr/bin/env python3
"""Publish a map-coordinate occupancy display and a ground-truth robot marker for RViz."""
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid,Odometry
from visualization_msgs.msg import Marker

class MapDisplay(Node):
    """Display the same immutable occupancy source as the planner; never generate sensor data."""
    def __init__(self):
        """Publish the latched map once and subscribe to actual odometry for robot visualization."""
        super().__init__('sentry_map_display')
        root=Path(get_package_share_directory('trajectory_generation'))
        params=yaml.safe_load((root/'config/map_metadata.yaml').read_text())['trajectory_generation']['ros__parameters']
        img=np.array(Image.open(root/'map/occfinal.png').convert('L'))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=params['planner.map_resolution']
        grid.info.height,grid.info.width=img.shape
        grid.info.origin.position.x=params['planner.map_lower_x'];grid.info.origin.position.y=params['planner.map_lower_y'];grid.info.origin.orientation.w=1.
        grid.data=np.where(np.flipud(img)>10,100,0).astype(np.int8).flatten().tolist()
        self.map=self.create_publisher(OccupancyGrid,'/sim/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));self.map.publish(grid)
        self.marker=self.create_publisher(Marker,'/sim/robot',10);self.count=0
        self.create_subscription(Odometry,'/localization/odometry',self.receive,10)
    def receive(self,odom):
        """Draw actual robot pose at 10 Hz; the source remains Gazebo ground truth."""
        self.count+=1
        if self.count%5:return
        m=Marker();m.header=odom.header;m.ns='sentry';m.id=0;m.type=Marker.CUBE;m.action=Marker.ADD
        m.pose=odom.pose.pose;m.pose.position.z+=.25
        m.scale.x=.7;m.scale.y=.5;m.scale.z=.3
        m.color.r=.05;m.color.g=.4;m.color.b=.9;m.color.a=1.
        self.marker.publish(m)

def main():
    """Keep transient-local map available for RViz restarts until launch shuts down."""
    rclpy.init();node=MapDisplay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
