#!/usr/bin/env python3
"""Generate a compact 1 m obstacle mesh from the planner's untouched raw occupancy map."""
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from PIL import Image

def rectangles(mask):
    """Merge equal horizontal occupied runs across rows, preserving every source cell."""
    active={};result=[]
    for row in range(mask.shape[0]+1):
        line=mask[row] if row<mask.shape[0] else np.zeros(mask.shape[1],dtype=bool)
        padded=np.r_[False,line,False].astype(int);changes=np.diff(padded)
        runs=list(zip(np.where(changes==1)[0],np.where(changes==-1)[0]))
        current=set(runs)
        for run,start in active.items():
            if run not in current:result.append((start,row,run[0],run[1]))
        active={run:active.get(run,row) for run in runs}
    return result

def write_mesh(path,rects,resolution,lower_x,lower_y,rows):
    """Write merged cuboids in map coordinates as one COLLADA mesh to reduce draw calls."""
    vertices=[];triangles=[]
    faces=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    for r0,r1,c0,c1 in rects:
        x0=lower_x+c0*resolution;x1=lower_x+c1*resolution
        y0=lower_y+(rows-r1)*resolution;y1=lower_y+(rows-r0)*resolution
        start=len(vertices)
        vertices.extend([(x0,y0,0),(x1,y0,0),(x1,y1,0),(x0,y1,0),(x0,y0,1),(x1,y0,1),(x1,y1,1),(x0,y1,1)])
        triangles.extend(tuple(start+i for i in face) for face in faces)
    coords=' '.join(str(v) for xyz in vertices for v in xyz)
    indices=' '.join(str(v) for face in triangles for v in face)
    path.write_text(f'''<?xml version="1.0"?>
<!-- Generated raw-map obstacles; geometry is planar occupancy extruded to 1 m, not real terrain. -->
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_effects><effect id="wall-effect"><profile_COMMON><technique sid="common"><lambert><diffuse><color>0.38 0.48 0.58 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="wall-material"><instance_effect url="#wall-effect"/></material></library_materials>
<library_geometries><geometry id="obstacles"><mesh>
<source id="positions"><float_array id="coords" count="{len(vertices)*3}">{coords}</float_array>
<technique_common><accessor source="#coords" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="verts"><input semantic="POSITION" source="#positions"/></vertices>
<triangles material="wall" count="{len(triangles)}"><input semantic="VERTEX" source="#verts" offset="0"/><p>{indices}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="map"><instance_geometry url="#obstacles"><bind_material><technique_common><instance_material symbol="wall" target="#wall-material"/></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')

def main():
    """Validate metadata, generate deterministic geometry and record exact map provenance."""
    pkg=Path(__file__).resolve().parents[1];planner=pkg.parent/'trajectory_generation'
    meta=yaml.safe_load((planner/'config/map_metadata.yaml').read_text())['trajectory_generation']['ros__parameters']
    raw=yaml.safe_load((planner/'map/map_meta.yaml').read_text())
    res=meta['planner.map_resolution'];lx=meta['planner.map_lower_x'];ly=meta['planner.map_lower_y']
    assert (res,lx,ly)==(raw['resolution'],raw['map_lower_x'],raw['map_lower_y'])
    image=np.array(Image.open(planner/'map/occfinal.png').convert('L'))
    rows,cols=image.shape;assert (rows,cols)==(raw['rows'],raw['cols'])
    mask=image>10;rects=rectangles(mask);coverage=np.zeros_like(image,dtype=np.uint16)
    for r0,r1,c0,c1 in rects:coverage[r0:r1,c0:c1]+=1
    assert np.array_equal(coverage,mask.astype(np.uint16))
    folder=pkg/'models/planning_map';(folder/'meshes').mkdir(parents=True,exist_ok=True)
    write_mesh(folder/'meshes/obstacles.dae',rects,res,lx,ly,rows)
    (folder/'model.config').write_text('<?xml version="1.0"?>\n<!-- Generated planning-map presentation asset. -->\n<model><name>planning_map</name><version>1.0</version><sdf version="1.8">model.sdf</sdf></model>\n')
    (folder/'model.sdf').write_text('<?xml version="1.0"?>\n<!-- Raw occupancy mesh without robot-radius inflation or contact dynamics. -->\n<sdf version="1.8"><model name="planning_map"><static>true</static><link name="map"><visual name="obstacles"><geometry><mesh><uri>meshes/obstacles.dae</uri></mesh></geometry><material><ambient>0.38 0.48 0.58 1</ambient><diffuse>0.38 0.48 0.58 1</diffuse></material></visual></link></model></sdf>\n')
    world=ET.parse(pkg/'worlds/lidar_probe.sdf');w=world.getroot().find('world');w.set('name','sentry_planning')
    for model in list(w.findall('model')):w.remove(model)
    include=w.find('include');ET.SubElement(include,'name').text='sentry'
    ET.SubElement(include,'pose').text='-0.919 -4.454 0 0 0 0'
    w.append(ET.fromstring('<include><uri>model://planning_map</uri></include>'))
    w.append(ET.fromstring(f'<model name="ground"><static>true</static><pose>{lx+10} {ly+10} -0.03 0 0 0</pose><link name="ground"><visual name="floor"><geometry><box><size>20 20 0.05</size></box></geometry><material><ambient>0.8 0.8 0.78 1</ambient><diffuse>0.8 0.8 0.78 1</diffuse></material></visual></link></model>'))
    w.find("gui/plugin[@filename='MinimalScene']/camera_pose").text='4 -10 12 0 0.92 2.2'
    world.getroot().insert(0,ET.Comment('Generated by map_assets.py: map-aligned ideal planar scene, real lidar, no contact dynamics.'))
    ET.indent(world);world.write(pkg/'worlds/planning.sdf',encoding='unicode',xml_declaration=True)
    # Keep source hashes and cell ranges sufficient for independent reconstruction tests.
    cells=np.argwhere(mask);landmarks=[]
    for i in [len(cells)//4,len(cells)//2,3*len(cells)//4]:
        r,c=map(int,cells[i]);landmarks.append({'row':r,'column':c,'map_x':lx+(c+.5)*res,'map_y':ly+(rows-r-.5)*res})
    record={'description':'Uninflated raw-map geometry generation and exact per-cell coverage verification.',
            'resolution':res,'lower':[lx,ly],'shape':[rows,cols],'occupied_cells':int(mask.sum()),
            'cuboids':len(rects),'coverage_mismatches':int(np.count_nonzero(coverage!=mask)),
            'rectangles':[[int(x) for x in rect] for rect in rects],'landmarks':landmarks,'source_sha256':{
                str(p.relative_to(planner)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [planner/'map'/n for n in ['occfinal.png','bevfinal.png','occtopo.png','map_meta.yaml']]+[planner/'config/map_metadata.yaml']}}
    (pkg/'config/map_manifest.json').write_text(json.dumps(record,indent=2))
    print(json.dumps({k:v for k,v in record.items() if k!='rectangles'},indent=2))

if __name__=='__main__':main()
