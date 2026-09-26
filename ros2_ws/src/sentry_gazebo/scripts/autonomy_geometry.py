#!/usr/bin/env python3
"""Check references and MPC predictions against the original planner's occupied footprint grid."""
from pathlib import Path
import math
import numpy as np
import yaml
from PIL import Image
from ament_index_python.packages import get_package_share_directory

class StaticSafetyGrid:
    # Accept an explicit mask for isolated tests; runtime loads immutable planner maps and metadata.
    def __init__(self, mask, resolution=.05, lower=(-13.394,-12.079)):
        self.mask=np.asarray(mask,dtype=bool)
        self.resolution=float(resolution)
        self.lower=lower

    # Preserve the production integer disk inflation rule; autonomous planning may reserve extra clearance.
    @classmethod
    def load(cls):
        share=Path(get_package_share_directory('trajectory_generation'))
        metadata=yaml.safe_load((share/'config/map_metadata.yaml').read_text())
        params=metadata['trajectory_generation']['ros__parameters']
        resolution=params['planner.map_resolution']
        radius=int(params['planner.robot_radius']/resolution)
        raw=np.asarray(Image.open(share/'map/occfinal.png'))
        if raw.ndim==3:raw=raw[:,:,0]
        occupied=raw>10;mask=occupied.copy();height,width=mask.shape
        for dr in range(-radius,radius+1):
            for dc in range(-radius,radius+1):
                if dr*dr+dc*dc>radius*radius:continue
                r0=max(0,dr);r1=min(height,height+dr);c0=max(0,dc);c1=min(width,width+dc)
                mask[r0:r1,c0:c1]|=occupied[r0-dr:r1-dr,c0-dc:c1-dc]
        return cls(mask,resolution,(params['planner.map_lower_x'],params['planner.map_lower_y']))

    # Off-map or nonfinite coordinates are occupied, never clipped to a convenient free cell.
    def points_free(self, points):
        points=np.asarray(points,dtype=float).reshape(-1,2)
        if not len(points) or not np.isfinite(points).all():return False
        col=np.floor((points[:,0]-self.lower[0])/self.resolution).astype(int)
        row=self.mask.shape[0]-1-np.floor((points[:,1]-self.lower[1])/self.resolution).astype(int)
        inside=(row>=0)&(row<self.mask.shape[0])&(col>=0)&(col<self.mask.shape[1])
        return bool(inside.all() and not self.mask[row,col].any())

    # Bound spatial spacing by the analytic derivative coefficient bound, including every segment endpoint.
    def trajectory_free(self, message):
        for i,duration in enumerate(message.duration):
            x=np.array(message.coef_x[4*i:4*i+4],dtype=float)
            y=np.array(message.coef_y[4*i:4*i+4],dtype=float)
            bound=math.hypot(3*abs(x[0])*duration**2+2*abs(x[1])*duration+abs(x[2]),
                             3*abs(y[0])*duration**2+2*abs(y[1])*duration+abs(y[2]))
            count=max(2,math.ceil(duration*bound/.02)+1)
            if count>50000:return False
            times=np.linspace(0,duration,count)
            if not self.points_free(np.column_stack((np.polyval(x,times),np.polyval(y,times)))):return False
        return True

    # Inspect the first 0.5 seconds of actual MPC predictions, including between adjacent poses.
    def prediction_free(self, message):
        if message.header.frame_id!='map' or len(message.poses)<2:return False
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in message.poses[:6]])
        if not np.isfinite(points).all():return False
        for a,b in zip(points,points[1:]):
            count=max(2,math.ceil(float(np.linalg.norm(b-a))/.02)+1)
            if count>1000 or not self.points_free(np.linspace(a,b,count)):return False
        return True
