#!/usr/bin/env python3
"""Audit measured/interpolated motion against production occupancy and the time-varying real test box."""
import argparse,json,math
from pathlib import Path
import numpy as np
from autonomy_geometry import StaticSafetyGrid
from dynamic_replan_check import clearance

def main():
    """Interpolation bounds spatial checking density without pretending to add sensor measurements."""
    p=argparse.ArgumentParser();p.add_argument('inputs',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
    grid=StaticSafetyGrid.load();records=[]
    for file in a.inputs:
        data=json.loads(Path(file).read_text());samples=data['samples'];occupied=0;gap=math.inf;count=0
        for first,last in zip(samples,samples[1:]):
            distance=math.hypot(last['x']-first['x'],last['y']-first['y'])
            n=max(2,math.ceil(distance/.02)+1)
            for t in np.linspace(0,1,n):
                x=first['x']+t*(last['x']-first['x']);y=first['y']+t*(last['y']-first['y'])
                sim=first['sim']+t*(last['sim']-first['sim'])
                count+=1
                if not grid.points_free([[x,y]]):occupied+=1
                box=data.get('physical_box')
                if box and sim>=data['change_sim']:
                    center=box['center']
                    if sim>=data.get('obstacle_move',{}).get('sim',math.inf):center=data['obstacle_move']['new_center']
                    gap=min(gap,clearance(x,y,center,box['size']))
        record={'source':file,'interpolated_samples':count,'occupied_samples':occupied,
            'min_box_clearance':None if math.isinf(gap) else gap,'passed':occupied==0 and gap>=.05}
        records.append(record)
    Path(a.output).write_text(json.dumps(records,indent=2));print(json.dumps(records))
    if not all(r['passed'] for r in records):raise SystemExit(1)

if __name__=='__main__':main()
