#!/usr/bin/env python3
"""Publish a map-coordinate occupancy display and a ground-truth robot marker for RViz."""
from pathlib import Path
import copy,hashlib,json
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid,Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from simulation_map import map_paths, map_filenames

class MapDisplay(Node):
    """Display the same immutable occupancy source as the planner; never generate sensor data."""
    def __init__(self):
        """Publish the latched map once and subscribe to actual odometry for robot visualization."""
        super().__init__('sentry_map_display')
        root,maps,metadata_path,manifest=map_paths()
        expected=json.loads(manifest.read_text())['source_sha256']
        for name,digest in expected.items():
            if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Planner map changed: regenerate scene before launch: '+name)
        params=yaml.safe_load(metadata_path.read_text())['trajectory_generation']['ros__parameters']
        files=map_filenames(maps)
        img=np.array(Image.open(maps/files['occupancy']).convert('L'))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=params['planner.map_resolution']
        grid.info.height,grid.info.width=img.shape
        grid.info.origin.position.x=params['planner.map_lower_x'];grid.info.origin.position.y=params['planner.map_lower_y'];grid.info.origin.orientation.w=1.
        grid.data=np.where(np.flipud(img)>10,100,0).astype(np.int8).flatten().tolist()
        self.map=self.create_publisher(OccupancyGrid,'/sim/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));self.map.publish(grid)
        skeleton=np.asarray(Image.open(maps/files['topology']).convert('L'))>10
        height,width=skeleton.shape
        rows,cols=np.where(skeleton)
        lines=Marker();lines.header.frame_id='map';lines.ns='upstream_rmuc2024_skeleton';lines.id=0
        lines.type=Marker.LINE_LIST;lines.action=Marker.ADD;lines.pose.orientation.w=1.
        lines.scale.x=.035;lines.color.r=0.;lines.color.g=.8;lines.color.b=1.;lines.color.a=1.
        for row,col in zip(rows,cols):
            x=params['planner.map_lower_x']+(col+.5)*grid.info.resolution
            y=params['planner.map_lower_y']+(height-row-.5)*grid.info.resolution
            for dr,dc in ((-1,0),(-1,1),(0,1),(1,1)):
                rr,cc=row+dr,col+dc
                if 0<=rr<height and 0<=cc<width and skeleton[rr,cc]:
                    x2=params['planner.map_lower_x']+(cc+.5)*grid.info.resolution
                    y2=params['planner.map_lower_y']+(height-rr-.5)*grid.info.resolution
                    lines.points.extend((Point(x=float(x),y=float(y),z=.08),Point(x=float(x2),y=float(y2),z=.08)))
        self.roads=self.create_publisher(Marker,'/sim/road_topology',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.roads.publish(lines)
        self.marker=self.create_publisher(Marker,'/sim/robot',10);self.count=0
        self.create_subscription(Odometry,'/localization/odometry',self.receive,10)
    def receive(self,odom):
        """Draw actual robot pose at 10 Hz; the source remains Gazebo ground truth."""
        self.count+=1
        if self.count%5:return
        m=Marker();m.header=odom.header;m.ns='sentry';m.id=0;m.type=Marker.CUBE;m.action=Marker.ADD
        m.pose=copy.deepcopy(odom.pose.pose);m.pose.position.z+=.25
        m.scale.x=.7;m.scale.y=.5;m.scale.z=.3
        m.color.r=.05;m.color.g=.4;m.color.b=.9;m.color.a=1.
        self.marker.publish(m)

def main():
    """Keep transient-local map available for RViz restarts until launch shuts down."""
    rclpy.init();node=MapDisplay()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
