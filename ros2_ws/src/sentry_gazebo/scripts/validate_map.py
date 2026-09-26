#!/usr/bin/env python3
"""Independently rasterize generated mesh bounds and compare live map/cloud coordinates."""
import json,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from PIL import Image
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

def main():
    """Check all 160k cells, three landmarks and actual live cloud/map messages."""
    root=Path(__file__).resolve().parents[4];pkg=root/'ros2_ws/src/sentry_gazebo'
    meta=json.loads((pkg/'config/map_manifest.json').read_text());res=meta['resolution'];lx,ly=meta['lower'];rows,cols=meta['shape']
    tree=ET.parse(pkg/'models/planning_map/meshes/obstacles.dae')
    floats=tree.find('.//{*}float_array').text
    boxes=np.fromstring(floats,sep=' ').reshape(-1,8,3);coverage=np.zeros((rows,cols),dtype=np.uint16)
    bounds=[]
    for box in boxes:
        lo=box.min(axis=0);hi=box.max(axis=0)
        c0,c1=np.rint((np.array([lo[0],hi[0]])-lx)/res).astype(int)
        r0,r1=rows-np.rint((np.array([hi[1],lo[1]])-ly)/res).astype(int)
        assert abs(lo[2])<1e-10 and abs(hi[2]-1)<1e-10
        coverage[r0:r1,c0:c1]+=1;bounds.append((lo,hi))
    source=np.array(Image.open(root/'ros2_ws/src/trajectory_generation/map/occfinal.png').convert('L'))>10
    assert np.array_equal(coverage,source.astype(np.uint16))
    landmarks=[]
    for mark in meta['landmarks']:
        point=np.array([mark['map_x'],mark['map_y'],.5])
        assert any(np.all(point>=lo-1e-9) and np.all(point<=hi+1e-9) for lo,hi in bounds)
        c=int(np.floor((point[0]-lx)/res));r=rows-1-int(np.floor((point[1]-ly)/res))
        assert [r,c]==[mark['row'],mark['column']]
        expected=np.array([lx+(c+.5)*res,ly+(rows-r-.5)*res])
        error=float(np.linalg.norm(point[:2]-expected));assert error<=.05
        landmarks.append(dict(mark,error_m=error))
    rclpy.init();n=rclpy.create_node('map_alignment_check');messages={}
    n.create_subscription(OccupancyGrid,'/sim/map',lambda m:messages.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(PointCloud2,'/aligned_points',lambda m:messages.update(cloud=m),qos_profile_sensor_data)
    n.create_subscription(Odometry,'/localization/odometry',lambda m:messages.update(odom=m),10)
    end=time.monotonic()+5
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    assert len(messages)==3,messages.keys()
    grid=messages['map'];assert grid.header.frame_id=='map'
    assert abs(grid.info.resolution-res)<1e-8
    assert abs(grid.info.origin.position.x-lx)<1e-9 and abs(grid.info.origin.position.y-ly)<1e-9
    assert np.array_equal(np.array(grid.data).reshape(rows,cols),np.flipud(source)*100)
    cloud=messages['cloud'];data=point_cloud2.read_points(cloud,field_names=('x','y','z'))
    xyz=np.column_stack([data[k].reshape(-1) for k in ('x','y','z')]);assert len(xyz)>100 and np.isfinite(xyz).all()
    # Horizontal returns on 1 m walls must land within a cell diagonal of source occupancy.
    sample=xyz[(xyz[:,2]>.1)&(xyz[:,2]<.95)][::4,:2]
    occupied=np.argwhere(source)
    centers=np.column_stack([lx+(occupied[:,1]+.5)*res,ly+(rows-occupied[:,0]-.5)*res])
    distances=np.sqrt(((sample[:,None,:]-centers[None,:,:])**2).sum(axis=2)).min(axis=1)
    assert len(distances)>20 and float(np.quantile(distances,.95))<=.1
    p=messages['odom'].pose.pose.position
    assert abs(p.x+.919)<1e-8 and abs(p.y+4.454)<1e-8
    result={'passed':True,'checked_cells':rows*cols,'coverage_mismatches':0,'mesh_cuboids':len(boxes),
            'landmarks':landmarks,'live_map_matches':True,'spawn':[p.x,p.y,p.z],
            'real_cloud_points':len(xyz),'cloud_occupancy_distance_p95':float(np.quantile(distances,.95)),
            'cloud_occupancy_distance_max':float(distances.max())}
    (root/'docs/simulation/part2/evidence/map-validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
