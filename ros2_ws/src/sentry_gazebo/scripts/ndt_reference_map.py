#!/usr/bin/env python3
"""Create an NDT reference map from the same raw occupied cells as the extruded Gazebo scene, never a sensor stream."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import yaml
from PIL import Image

# Sample exposed cell faces and the ground plane in the original map coordinates.
def generate(root,output,map_directory=None,metadata_file=None):
    source=root/'ros2_ws/src/trajectory_generation'
    map_directory=map_directory or source/'map'
    metadata_file=metadata_file or source/'config/map_metadata.yaml'
    metadata=yaml.safe_load(metadata_file.read_text())['trajectory_generation']['ros__parameters']
    map_manifest=map_directory/'map_manifest.json'
    map_files=json.loads(map_manifest.read_text()).get('map_files',{}) if map_manifest.is_file() else {}
    occupancy_file=map_directory/map_files.get('occupancy','occfinal.png')
    raw=np.asarray(Image.open(occupancy_file))
    if raw.ndim==3:raw=raw[:,:,0]
    occupied=raw>10;h,w=occupied.shape;r=metadata['planner.map_resolution']
    lower=np.array([metadata['planner.map_lower_x'],metadata['planner.map_lower_y']])
    faces=[];zs=np.arange(0,1.001,.05)
    for row,col in zip(*np.where(occupied)):
        x,y=lower+np.array([col,h-1-row])*r
        for dr,dc,side in [(-1,0,'north'),(1,0,'south'),(0,-1,'west'),(0,1,'east')]:
            rr,cc=row+dr,col+dc
            if 0<=rr<h and 0<=cc<w and occupied[rr,cc]:continue
            for u in (0.,.5,1.):
                xx=x+(r if side=='east' else 0. if side=='west' else u*r)
                yy=y+(r if side=='north' else 0. if side=='south' else u*r)
                faces.extend((xx,yy,float(z),0.) for z in zs)
    # Reference floor points are geometry, not fabricated observations.
    for row in range(0,h,2):
        for col in range(0,w,2):
            if not occupied[row,col]:faces.append((lower[0]+(col+.5)*r,lower[1]+(h-row-.5)*r,0.,0.))
    points=np.unique(np.round(np.asarray(faces),5),axis=0).astype('<f4')
    header=('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode()
    output.parent.mkdir(parents=True,exist_ok=True);output.write_bytes(header+points.tobytes())
    provenance={'purpose':'reference geometry only; lidar observations remain Gazebo sensor data',
        'occupancy_file':occupancy_file.name,
        'occupancy_sha256':hashlib.sha256(occupancy_file.read_bytes()).hexdigest(),
        'pcd_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'points':len(points),'height_m':1.,
        'resolution_m':r,'lower':lower.tolist(),'intensity':'zero placeholder, not a modeled sensor intensity'}
    output.with_suffix('.json').write_text(json.dumps(provenance,indent=2));print(json.dumps(provenance))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--map-directory',type=Path);parser.add_argument('--metadata',type=Path);args=parser.parse_args()
    generate(Path(__file__).resolve().parents[4],args.output,args.map_directory,args.metadata)
