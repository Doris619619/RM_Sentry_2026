#!/usr/bin/env python3
"""Measure goal replacement and physical Gazebo obstacle replanning using actual sensor and truth data."""
import argparse,json,math,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from closed_loop_check import ClosedLoop,autonomy_cases,ROOT

# Mutate only the named test model through the simulator's real entity service.
def service(action,kind,request):
    result=subprocess.run(['ign','service','-s','/world/sentry_planning/'+action,
        '--reqtype','ignition.msgs.'+kind,'--reptype','ignition.msgs.Boolean',
        '--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
    if result.returncode or 'true' not in result.stdout:raise RuntimeError(action+' failed: '+result.stdout)

# Publish a map-frame goal at the current simulation clock.
def goal(live,xy):
    m=PoseStamped();m.header.frame_id='map';m.header.stamp=live.node.get_clock().now().to_msg()
    m.pose.position.x=float(xy[0]);m.pose.position.y=float(xy[1]);m.pose.orientation.w=1.
    live.pub.publish(m)

# Spin until actual callbacks satisfy the condition, with a bounded wall deadline.
def wait(live,predicate,timeout):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        live.spin(.03)
        if predicate():return
    raise RuntimeError('condition timeout: '+str(live.status))

# Check the swept circular footprint against the known real test box, independently of controller status.
def clearance(x,y,center,size):
    return math.hypot(max(abs(x-center[0])-size[0]/2,0),max(abs(y-center[1])-size[1]/2,0))-.35

def main():
    """Use set_pose only before a case; obstacle changes during motion never teleport the robot."""
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=['goal_change','obstacle','moving_obstacle','blocked'],required=True)
    parser.add_argument('--output',required=True);args=parser.parse_args()
    rclpy.init();live=ClosedLoop();result={'case':args.case};spawned=False
    center=(-3.9,-2.35);size=(24.,.4) if args.case=='blocked' else (.4,.4);returns=[];clouds=[];history=[]
    # Count real map-frame lidar returns on the newly introduced box.
    def cloud(msg):
        points=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
        xyz=np.column_stack([points[n].reshape(-1) for n in ('x','y','z')])
        if len(xyz):
            keep=(abs(xyz[:,0]-center[0])<.23)&(abs(xyz[:,1]-center[1])<.23)&(xyz[:,2]>.2)
            if keep.any():
                returns.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'points':xyz[keep].tolist()})
    live.node.create_subscription(PointCloud2,'/aligned_points',cloud,qos_profile_sensor_data)
    live.node.create_subscription(String,'/sim/autonomy/status',lambda m:history.append(json.loads(m.data)),10)
    try:
        wait(live,lambda:live.odom is not None and bool(live.status),30)
        start,original=autonomy_cases()[0][0];live.prepare(start);goal(live,original)
        wait(live,lambda:live.samples and live.samples[-1]['speed']>.1,20)
        wait(live,lambda:math.hypot(live.samples[-1]['x']-start[0],live.samples[-1]['y']-start[1])>.5,15)
        before=len(live.paths);stamp=live.samples[-1]['sim'];target=original
        if args.case=='goal_change':
            target=(-4.719,1.096);goal(live,target)
        else:
            sdf='<sdf version="1.8"><model name="qa_dynamic_box"><static>true</static><pose>-3.9 -2.35 .5 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>.4 .4 1</size></box></geometry></collision><visual name="visual"><geometry><box><size>.4 .4 1</size></box></geometry><material><ambient>1 .4 0 1</ambient><diffuse>1 .4 0 1</diffuse></material></visual></link></model></sdf>'
            if args.case=='blocked':sdf=sdf.replace('.4 .4 1','24 .4 1')
            service('create','EntityFactory','sdf: '+json.dumps(sdf));spawned=True
            result['physical_box']={'center':list(center),'size':list(size)}
        result['change_sim']=stamp;result['trajectories_before_change']=before
        end=time.monotonic()+160
        min_clearance=math.inf
        moved=False
        while time.monotonic()<end:
            live.spin(.04)
            if args.case=='moving_obstacle' and not moved and live.samples[-1]['sim']-stamp>3.:
                center=(-4.1,-2.55)
                service('set_pose','Pose','name: "qa_dynamic_box" position: {x: -4.1 y: -2.55 z: .5} orientation: {w: 1}')
                result['obstacle_move']={'sim':live.samples[-1]['sim'],'new_center':list(center)}
                moved=True
            if args.case in ('obstacle','moving_obstacle','blocked'):
                p=live.samples[-1];gap=clearance(p['x'],p['y'],center,size);min_clearance=min(min_clearance,gap)
                if gap<.05:raise RuntimeError('independent obstacle safety stop: footprint within 0.05 m')
            if live.status.get('state')=='arrived':break
            if live.status.get('state')=='stopped':
                if args.case=='blocked':break
                raise RuntimeError('stopped: '+live.status.get('reason',''))
        p=live.samples[-1];error=math.hypot(p['x']-target[0],p['y']-target[1])
        result.update(endpoint_error=error,new_trajectories=len(live.paths)-before,
            actual_obstacle_returns=len(returns),min_box_clearance=None if math.isinf(min_clearance) else min_clearance,
            target=list(target),status=live.status)
        result['passed']=live.status.get('state')=='arrived' and error<=.15 and len(live.paths)>before and (args.case=='goal_change' or bool(returns))
        if args.case=='blocked':
            before_stop=live.samples[-1];live.spin(1.2);after_stop=live.samples[-1]
            result['stopped_drift']=math.hypot(after_stop['x']-before_stop['x'],after_stop['y']-before_stop['y'])
            result['passed']=live.status.get('state')=='stopped' and bool(returns) and result['stopped_drift']<.03 and after_stop['speed']<.02
        if not result['passed']:raise RuntimeError('did not demonstrate new safe route and arrival')
    except Exception as error:
        result['passed']=False;result['error']=str(error)
    finally:
        live.mode.publish(String(data='stop'));live.spin(.3)
        if spawned:service('remove','Entity','name: "qa_dynamic_box" type: MODEL')
        result['samples']=live.samples;result['trajectories']=live.paths;result['lidar_box_observations']=returns;result['status_history']=history
        Path(args.output).write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','trajectories','lidar_box_observations','status_history')}))
        live.node.destroy_node();rclpy.shutdown()
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
