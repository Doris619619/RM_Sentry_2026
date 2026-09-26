#!/usr/bin/env python3
"""Register and gracefully stop only this repository's verified autonomous launch for restart."""
import argparse,json,os,signal
from pathlib import Path
import psutil

# Require a matching process identity, owner and exact launch arguments before sending any signal.
def stop_owned(path,root):
    record=json.loads(path.read_text());process=psutil.Process(record['pid'])
    if record['root']!=str(root) or process.uids().real!=os.getuid():
        raise RuntimeError('launch ownership mismatch')
    if abs(process.create_time()-record['created'])>.001:
        raise RuntimeError('PID was reused; refusing to stop another process')
    command=process.cmdline()
    if not all(value in command for value in ['launch','sentry_gazebo','autonomy.launch.py']):
        raise RuntimeError('recorded process is not the autonomous launch')
    descendants=process.children(recursive=True)
    process.send_signal(signal.SIGINT)
    _,alive=psutil.wait_procs([process]+descendants,timeout=20)
    remaining=[]
    for child in alive:
        try:
            if child.is_running() and child.status()!=psutil.STATUS_ZOMBIE:remaining.append(child.pid)
        except psutil.NoSuchProcess:pass
    if remaining:raise RuntimeError('simulation did not stop cleanly: '+str(remaining))
    path.unlink()

# A stale identity is never treated as permission to signal a reused PID.
def main():
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['register','stop'])
    args=parser.parse_args();root=Path(__file__).resolve().parents[4]
    directory=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))
    path=directory/('sentry-autonomy-'+str(os.getuid())+'-'+os.environ.get('ROS_DOMAIN_ID','26')+'.json')
    if args.action=='register':
        parent=psutil.Process(os.getppid())
        record={'pid':parent.pid,'created':parent.create_time(),'root':str(root)}
        path.write_text(json.dumps(record));path.chmod(0o600)
    else:
        if not path.is_file():raise SystemExit('没有登记中的自主仿真；请使用 start 启动。')
        stop_owned(path,root)
        print('旧仿真已完整退出，开始创建新场景。')

if __name__=='__main__':main()
