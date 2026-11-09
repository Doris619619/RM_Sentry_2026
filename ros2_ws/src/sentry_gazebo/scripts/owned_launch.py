#!/usr/bin/env python3
"""Register and gracefully stop only this repository's verified autonomous launch for restart."""
import argparse,json,os,signal,shlex,getpass
from pathlib import Path
import psutil

def wait_for_stop(processes):
    """Allow orderly shutdown, then terminate only already verified surviving children."""
    _,alive=psutil.wait_procs(processes,timeout=20)
    for process in alive:
        try:
            if process.status()!=psutil.STATUS_ZOMBIE:process.terminate()
        except psutil.NoSuchProcess:pass
    _,alive=psutil.wait_procs(alive,timeout=5)
    remaining=[]
    for process in alive:
        try:
            if process.is_running() and process.status()!=psutil.STATUS_ZOMBIE:remaining.append(process.pid)
        except psutil.NoSuchProcess:pass
    if remaining:raise RuntimeError('simulation did not stop cleanly: '+str(remaining))

def stop_orphans(root):
    """Recover only this checkout's simulator processes if its launch has disappeared."""
    prefix=str(root/'ros2_ws/install_sentry_autonomy')+'/'
    localization=str(root/'ros2_ws/install_sim_deps/hdl_localization/lib/hdl_localization')+'/'
    partition='sentry_basic_'+os.environ.get('USER',getpass.getuser())+'_'+os.environ.get('ROS_DOMAIN_ID','26')
    owned={}
    for process in psutil.process_iter(['uids','cmdline']):
        try:
            command=process.info['cmdline'] or []
            if process.info['uids'].real!=os.getuid() or not command:continue
            env=process.environ()
            if env.get('IGN_PARTITION')!=partition or not env.get('SENTRY_DEPENDENCY_SETUP','').startswith(str(root)+'/'):continue
            local=any(arg.startswith(prefix) or arg.startswith(localization) for arg in command[:2])
            gazebo=shlex.split(command[0]) if command[0].startswith('ign gazebo ') else command
            gazebo=(len(gazebo)>2 and gazebo[:2]==['ign','gazebo'] and any(arg.startswith(prefix) for arg in gazebo))
            # Gazebo rewrites child argv to this title after its launcher exits.
            gazebo=bool(gazebo or (command[0] in ('ign gazebo server','ign gazebo gui') and env.get('SENTRY_LOCALIZATION') in ('truth','ndt')))
            bridge=command[0].endswith('/ros_gz_bridge/parameter_bridge')
            if not (local or gazebo or bridge):continue
            for child in [process]+process.children(recursive=True):
                if child.uids().real==os.getuid():owned[child.pid]=child
        except (psutil.Error,OSError,ValueError):continue
    for process in owned.values():
        try:process.send_signal(signal.SIGINT)
        except psutil.NoSuchProcess:pass
    wait_for_stop(list(owned.values()))
    if owned:print('已清理本项目遗留的仿真进程：'+str(sorted(owned)))

# Require a matching process identity, owner and exact launch arguments before sending any signal.
def stop_owned(path,root):
    record=json.loads(path.read_text())
    if record['root']!=str(root):
        raise RuntimeError('launch ownership mismatch')
    try:process=psutil.Process(record['pid'])
    except psutil.NoSuchProcess:
        stop_orphans(root)
        path.unlink()
        print('登记的仿真已退出，已清理旧记录。')
        return
    if process.uids().real!=os.getuid():
        raise RuntimeError('launch ownership mismatch')
    if abs(process.create_time()-record['created'])>.001:
        raise RuntimeError('PID was reused; refusing to stop another process')
    command=process.cmdline()
    if not all(value in command for value in ['launch','sentry_gazebo','autonomy.launch.py']):
        raise RuntimeError('recorded process is not the autonomous launch')
    descendants=process.children(recursive=True)
    process.send_signal(signal.SIGINT)
    wait_for_stop([process]+descendants)
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
        if path.is_file():stop_owned(path,root)
        else:stop_orphans(root)
        print('仿真已完整退出。')

if __name__=='__main__':main()
