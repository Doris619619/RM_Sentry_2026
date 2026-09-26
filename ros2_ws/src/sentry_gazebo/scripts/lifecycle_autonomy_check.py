#!/usr/bin/env python3
"""Verify complete autonomous-launch shutdown and a single-robot restart using real ROS/Gazebo state."""
import argparse,json,os,re,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

TOPICS=['/clock','/localization/odometry','/sim/lidar/native','/cmd_vel','/global_trajectory']

# Wait with callbacks and a hard wall deadline; retain live discovery instead of ROS CLI daemon caches.
def wait(node,predicate,timeout):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if predicate():return
    raise RuntimeError('lifecycle condition timed out')

# Query the actual scene service, counting exact model names rather than markers or odometry frames.
def robot_count():
    result=subprocess.run(['ign','service','-s','/world/sentry_planning/scene/info',
        '--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene','--timeout','3000','--req',''],
        capture_output=True,text=True,timeout=5)
    if result.returncode:raise RuntimeError('scene service failed')
    return len(re.findall(r'^  name: "sentry"$',result.stdout,re.M))

# Stop only the explicitly identified launch process group and leave the verified restart idle.
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pid',type=int,required=True)
    parser.add_argument('--restart-cli',action='store_true');parser.add_argument('--full-system',action='store_true')
    parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[4];launch=psutil.Process(args.pid)
    entry='gazebo_full.sh' if args.full_system else 'gazebo_autonomy.sh'
    if args.full_system:TOPICS.extend(['/sim/serial_cmd_vel','/referee/game_progress'])
    log_prefix='full-' if args.full_system else ''

    if 'autonomy.launch.py' not in ' '.join(launch.cmdline()) or os.getpgid(launch.pid)!=launch.pid:
        raise SystemExit('unexpected process group')
    rclpy.init();node=rclpy.create_node('autonomy_lifecycle_check')
    mode=node.create_publisher(String,'/sim/control_mode',10);odometry=[];commands=[];raw=[]
    node.create_subscription(Odometry,'/localization/odometry',lambda m:odometry.append(m),10)
    node.create_subscription(Twist,'/cmd_vel',lambda m:commands.append(m),10)
    node.create_subscription(Odometry,'/sim/ground_truth/odometry',lambda m:raw.append(m),10)
    records={};restarted=None
    try:
        wait(node,lambda:bool(odometry) and all(len(node.get_publishers_info_by_topic(topic))==1 for topic in TOPICS),15)
        mode.publish(String(data='stop'));until=time.monotonic()+.6
        wait(node,lambda:time.monotonic()>until,2)
        records['before']={topic:len(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
        records['robots_before']=robot_count()
        descendants=launch.children(recursive=True)
        if args.restart_cli:
            log=open(root/('docs/simulation/autonomy/evidence/'+log_prefix+'lifecycle-cli-restart-launch.log'),'w')
            restarted=subprocess.Popen(['bash',str(root/'scripts'/entry),'restart'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            Path('/tmp/sentry-autonomy.pid').write_text(str(restarted.pid))
            records['restart_method']=entry+' restart'
        else:
            launch.send_signal(signal.SIGINT)
        wait(node,lambda:all(not node.get_publishers_info_by_topic(topic) for topic in TOPICS),15)
        records['after_stop']={topic:len(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
        records['old_descendant_pids']=[process.pid for process in descendants]
        # DDS publishers can disappear before GUI teardown finishes; allow orderly exit.
        def remaining():
            alive=[]
            for process in descendants:
                try:
                    if process.is_running() and process.status()!=psutil.STATUS_ZOMBIE:alive.append(process.pid)
                except psutil.NoSuchProcess:pass
            return alive
        wait(node,lambda:not remaining(),15)
        records['old_descendants_remaining']=remaining()
        odometry.clear();commands.clear();raw.clear()
        if not args.restart_cli:
            log=open(root/('docs/simulation/autonomy/evidence/'+log_prefix+'lifecycle-restart-launch.log'),'w')
            restarted=subprocess.Popen(['bash',str(root/'scripts'/entry)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            Path('/tmp/sentry-autonomy.pid').write_text(str(restarted.pid))
        wait(node,lambda:bool(odometry) and all(len(node.get_publishers_info_by_topic(topic))==1 for topic in TOPICS),40)
        until=time.monotonic()+3
        wait(node,lambda:time.monotonic()>until,5)
        records['after_restart']={topic:len(node.get_publishers_info_by_topic(topic)) for topic in TOPICS}
        records['robots_after']=robot_count();records['restart_pid']=restarted.pid
        positions=[m.pose.pose.position for m in raw]
        if not positions:raise RuntimeError('no native pose evidence')
        records['native_pose_displacement']=max(((p.x-positions[0].x)**2+(p.y-positions[0].y)**2)**.5 for p in positions)
        records['native_settled_speed']=max((m.twist.twist.linear.x**2+m.twist.twist.linear.y**2)**.5 for m in raw[-50:])
        records['native_initial_twist_peak']=max((m.twist.twist.linear.x**2+m.twist.twist.linear.y**2)**.5 for m in raw)
        records['max_unrequested_speed']=max((m.twist.twist.linear.x**2+m.twist.twist.linear.y**2)**.5 for m in odometry)
        records['max_unrequested_command']=max((m.linear.x**2+m.linear.y**2)**.5 for m in commands)
        records['passed']=(records['robots_before']==records['robots_after']==1 and
                           records['native_pose_displacement']<.001 and records['native_settled_speed']<.02 and records['max_unrequested_speed']<(.03 if args.full_system else .001) and records['max_unrequested_command']<.001)
        if not records['passed']:raise RuntimeError('restart was not single and idle')
    except Exception as error:
        records['passed']=False;records['error']=str(error)
        if restarted and restarted.poll() is None:restarted.send_signal(signal.SIGINT)
        raise
    finally:
        Path(args.output).write_text(json.dumps(records,indent=2));print(json.dumps(records))
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
