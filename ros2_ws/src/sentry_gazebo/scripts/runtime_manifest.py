#!/usr/bin/env python3
"""Record the source and loaded-process provenance of an isolated simulation acceptance run."""
import argparse,hashlib,json,subprocess
from pathlib import Path
import psutil

# Hash actual files in bounded chunks so build provenance does not depend on filenames alone.
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

# Capture only this verified launch tree and repository assets; omit environment variables and credentials.
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pid',type=int,required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();root=Path(__file__).resolve().parents[4]
    launch=psutil.Process(args.pid)
    if 'autonomy.launch.py' not in ' '.join(launch.cmdline()):raise SystemExit('unexpected launch process')
    paths=set();processes=[]
    for p in [launch]+launch.children(recursive=True):
        try:
            processes.append({'pid':p.pid,'command':p.cmdline()})
            for arg in p.cmdline():
                f=Path(arg)
                if arg.startswith(str(root)) and f.is_file():paths.add(f)
            for mapping in p.memory_maps(grouped=True):
                if mapping.path.startswith(str(root)):
                    f=Path(mapping.path)
                    if f.is_file():paths.add(f)
        except psutil.Error:pass
    for package in ['sentry_gazebo','trajectory_generation','trajectory_tracking','decision_node','hdl_localization']:
        base=root/'ros2_ws/src'/package
        for name in ['config','scripts','launch','port/src','port/include','src']:
            directory=base/name
            if directory.exists():
                paths.update(f for f in directory.rglob('*') if f.is_file() and '__pycache__' not in str(f))
    maps=root/'ros2_ws/src/trajectory_generation/map'
    paths.update(maps/name for name in ['occfinal.png','bevfinal.png','occtopo.png'])
    paths.add(root/'ros2_ws/install_sentry_autonomy/sentry_gazebo/share/sentry_gazebo/maps/ndt_reference.pcd')
    output={'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
            'dirty':subprocess.check_output(['git','status','--short','--untracked-files=no'],cwd=root,text=True),
            'launch_pid':args.pid,'processes':processes,
            'sha256':{str(p.relative_to(root)):digest(p) for p in sorted(paths)}}
    Path(args.output).write_text(json.dumps(output,indent=2))
    print('Recorded',len(paths),'source/assets/binaries and',len(processes),'processes')

if __name__=='__main__':main()
