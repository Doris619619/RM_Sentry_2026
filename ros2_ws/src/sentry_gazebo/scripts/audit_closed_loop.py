#!/usr/bin/env python3
"""Independently audit recorded actual motion and all accepted polynomial references."""
import argparse,json,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from autonomy_geometry import StaticSafetyGrid

# Bound interpolation spacing; preserve raw samples and distinguish an interpolated audit from observations.
def audit(records):
    grid=StaticSafetyGrid.load()
    output=[]
    for record in records:
        points=np.asarray([[x['x'],x['y']] for x in record.get('samples',[])])
        motion_free=len(points)>1
        largest_gap=0.
        for a,b in zip(points,points[1:]):
            distance=float(np.linalg.norm(b-a));largest_gap=max(largest_gap,distance)
            if not grid.points_free(np.linspace(a,b,max(2,math.ceil(distance/.02)+1))):motion_free=False
        references=record.get('trajectories',[])
        paths_free=bool(references) and all(grid.trajectory_free(SimpleNamespace(
            duration=p['duration'],coef_x=p['x'],coef_y=p['y'])) for p in references)
        output.append({'case':record.get('case',record.get('goal')),'repeat':record.get('repeat',1),'actual_sample_max_gap_m':largest_gap,
                       'interpolated_motion_free':bool(motion_free),'all_reference_polynomials_free':bool(paths_free),
                       'passed':bool(record['passed'] and motion_free and paths_free)})
    return output

# Audit a completed recording without publishing or modifying any simulation state.
def main():
    parser=argparse.ArgumentParser();parser.add_argument('input');parser.add_argument('--output',required=True)
    args=parser.parse_args();result=audit(json.loads(Path(args.input).read_text()))
    Path(args.output).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    if not result or not all(x['passed'] for x in result):raise SystemExit(1)

if __name__=='__main__':main()
