#!/usr/bin/env python3
"""Record actual Gazebo/RViz windows with real keyboard motion and a real RViz goal click."""
import argparse,json,os,pty,select,signal,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from trajectory_generation.msg import TrajectoryPoly

def main():
    """Capture 90 seconds of two XWayland windows; preserve UI events and actual model state."""
    parser=argparse.ArgumentParser();parser.add_argument('--gazebo-window',required=True);parser.add_argument('--rviz-window',required=True)
    args=parser.parse_args();root=Path(__file__).resolve().parents[4];out=root/'docs/simulation/part2/evidence'
    for window,expected in [(args.gazebo_window,'Gazebo'),(args.rviz_window,'RViz')]:
        title=subprocess.check_output(['xdotool','getwindowname',window],text=True)
        assert expected in title,(window,title)
    pipeline=['gst-launch-1.0','-e','compositor','name=mix','background=black','sink_0::xpos=0','sink_1::xpos=960','!','videoconvert','!',
        'video/x-raw,format=I420,colorimetry=bt709,width=1920,height=816,framerate=10/1','!','vp8enc','deadline=1','cpu-used=8','threads=2','target-bitrate=2500000',
        '!','webmmux','!','filesink','location='+str(out/'demo-raw.webm')]
    for i,window in enumerate([args.gazebo_window,args.rviz_window]):
        pipeline+=['ximagesrc','xid='+window,'use-damage=false','show-pointer=true','num-buffers=900','!','video/x-raw,framerate=10/1','!',
            'videoscale','!','videoconvert','!','video/x-raw,format=I420,colorimetry=bt709,width=960,height=816,pixel-aspect-ratio=1/1','!','mix.sink_'+str(i)]
    capture=subprocess.Popen(pipeline,stdout=open(out/'demo-capture.log','w'),stderr=subprocess.STDOUT)
    started=time.monotonic();events=[];odom=[];goals=[];trajectories=[]
    rclpy.init();node=rclpy.create_node('demo_evidence')
    node.create_subscription(Odometry,'/localization/odometry',lambda m:odom.append(m),10)
    node.create_subscription(PoseStamped,'/goal',lambda m:goals.append(m),10)
    node.create_subscription(TrajectoryPoly,'/global_trajectory',lambda m:trajectories.append(m),10)
    master,slave=pty.openpty()
    keyboard=subprocess.Popen(['bash',str(root/'scripts/gazebo_planning.sh'),'keyboard'],stdin=slave,stdout=slave,stderr=slave,start_new_session=True);os.close(slave)
    def pump(seconds,key=None):
        """Drain feedback while sending repeat key events; no synthesized odometry is used."""
        end=time.monotonic()+seconds;next_key=0
        while time.monotonic()<end:
            if key is not None and time.monotonic()>=next_key:os.write(master,key);next_key=time.monotonic()+.05
            rclpy.spin_once(node,timeout_sec=.01)
    def event(name):
        """Record real timing and latest actual pose for caption provenance."""
        p=odom[-1].pose.pose if odom else None
        events.append({'name':name,'wall_seconds':time.monotonic()-started,
                       'pose':None if p is None else [p.position.x,p.position.y,p.orientation.z,p.orientation.w]})
    try:
        transcript=b'';deadline=time.monotonic()+8
        while '控制接口已连接'.encode() not in transcript:
            if select.select([master],[],[],.1)[0]:transcript+=os.read(master,4096)
            if time.monotonic()>deadline:raise RuntimeError('Keyboard did not connect')
        (out/'demo-keyboard.txt').write_bytes(transcript)
        pump(max(0,10-(time.monotonic()-started)));event('manual_rotation_start')
        pump(7.8,b'q');os.write(master,b' ');pump(3);event('manual_translation_start')
        pump(2,b'w');os.write(master,b' ');pump(.5);event('manual_motion_stopped')
        os.write(master,b'\x1b');keyboard.wait(timeout=5)
        pump(max(0,35-(time.monotonic()-started)));event('rviz_goal_click')
        subprocess.run(['xdotool','windowactivate','--sync',args.rviz_window],check=True)
        subprocess.run(['xdotool','mousemove','--window',args.rviz_window,'530','72','click','1'],check=True);pump(.3)
        subprocess.run(['xdotool','mousemove','--window',args.rviz_window,'1000','878','mousedown','1'],check=True);pump(.3)
        subprocess.run(['xdotool','mousemove_relative','--','30','0'],check=True);pump(.3)
        subprocess.run(['xdotool','mouseup','1'],check=True)
        pump(5);event('goal_result_display')
        assert goals and trajectories,'GUI goal or planning output absent'
        pump(max(0,90-(time.monotonic()-started)))
        capture.wait(timeout=10);assert capture.returncode==0
    finally:
        subprocess.run(['xdotool','mouseup','1'],check=False)
        if keyboard.poll() is None:
            try:os.write(master,b'\x1b');keyboard.wait(timeout=3)
            except Exception:os.killpg(keyboard.pid,signal.SIGTERM)
        os.close(master)
        if capture.poll() is None:capture.send_signal(signal.SIGINT);capture.wait(timeout=10)
        result={'events':events,'goals':[[m.pose.position.x,m.pose.position.y] for m in goals],
                'trajectory_messages':len(trajectories),'capture_exit':capture.returncode,
                'description':'Actual concurrent window capture; keyboard via terminal PTY and mouse via xdotool; no automatic tracking.'}
        (out/'demo-events.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
