#!/usr/bin/env python3
"""Accept the real decision -> planner -> MPC -> MCU PTY -> force plant -> NDT loop; publish only referee scenarios."""
import argparse,json,math,struct,time
from collections import deque
from pathlib import Path
import numpy as np,rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from hdl_localization.msg import ScanMatchingStatus
from geometry_msgs.msg import PointStamped,Twist
from std_msgs.msg import String,UInt8,UInt16,Bool
from trajectory_generation.msg import TrajectoryPoly
from autonomy_geometry import StaticSafetyGrid
from mcu_wire import crc8,crc16
from ndt_probe_check import stamp,yaw

class FullCheck:
    # Runtime graphs, wire frames, measured motion, and estimates are all recorded independently.
    def __init__(self):
        self.node=rclpy.create_node('full_system_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.scenario=self.node.create_publisher(String,'/sim/referee/scenario',10)
        self.mode=self.node.create_publisher(String,'/sim/control_mode',10)
        self.truth=deque(maxlen=200);self.pending=deque();self.samples=[];self.errors=[];self.registrations={};self.episode_hp=[]
        self.trajectories=[];self.arrivals=[];self.goals=[];self.wires=[];self.commands=[];self.status={};self.mcu={};self.hp=0;self.game=0
        self.node.create_subscription(Odometry,'/sim/ground_truth/odometry',self.on_truth,50)
        self.node.create_subscription(Odometry,'/localization/odometry',self.pending.append,20)
        self.node.create_subscription(ScanMatchingStatus,'/sim/ndt/status',lambda m:self.registrations.update({stamp(m):m.relative_pose.translation}),10)
        self.node.create_subscription(TrajectoryPoly,'/global_trajectory',self.on_path,10)
        self.node.create_subscription(String,'/sim/autonomy/status',lambda m:setattr(self,'status',json.loads(m.data)),10)
        self.node.create_subscription(String,'/sim/mcu/status',self.on_wire,10)
        self.node.create_subscription(PointStamped,'/sim/decision_goal',lambda m:self.goals.append([m.point.x,m.point.y]),10)
        self.node.create_subscription(Bool,'/dstar_status',self.on_arrival,10)
        self.node.create_subscription(UInt8,'/referee/game_progress',lambda m:setattr(self,'game',m.data),10)
        self.node.create_subscription(UInt16,'/robot/self_hp',lambda m:(setattr(self,'hp',m.data),self.episode_hp.append(m.data)),10)
        self.node.create_subscription(Twist,'/sim/guarded_cmd_vel',lambda m:self.commands.append([m.linear.x,m.linear.y,m.angular.z]),10)

    # Save measured physical state at bounded rate; never feed it into navigation.
    def on_truth(self,m):
        self.truth.append(m)
        if not self.samples or stamp(m)-self.samples[-1]['sim']>=.079:
            p=m.pose.pose.position;v=m.twist.twist
            self.samples.append({'sim':stamp(m),'x':p.x,'y':p.y,'z':p.z,'yaw':yaw(m.pose.pose.orientation),
                'speed':math.hypot(v.linear.x,v.linear.y),'wz':v.angular.z})

    # Preserve every accepted polynomial for independent spatial safety auditing.
    def on_path(self,m):
        if m.duration:self.trajectories.append({'duration':list(m.duration),'x':list(m.coef_x),'y':list(m.coef_y)})

    # Observe real serial decoder state without synthesizing MCU outputs.
    def on_wire(self,m):
        value=json.loads(m.data);self.mcu=value;self.wires.append(value)

    # Pair actual decision arrival feedback with the latest physical state.
    def on_arrival(self,m):
        if m.data and self.samples:self.arrivals.append(dict(self.samples[-1]))

    # Pair estimates with true poses only inside this evaluator.
    def spin(self,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(self.node,timeout_sec=.01)
            while self.pending and self.truth and stamp(self.pending[0])<=stamp(self.truth[-1]):
                m=self.pending.popleft();t=min(self.truth,key=lambda t:abs(stamp(t)-stamp(m)))
                if abs(stamp(t)-stamp(m))>.025:continue
                p,q=m.pose.pose.position,t.pose.pose.position
                raw=self.registrations.pop(stamp(m),None)
                self.errors.append({'sim':stamp(m),'xy':math.hypot(p.x-q.x,p.y-q.y),
                    'ndt_raw_xy':None if raw is None else math.hypot(raw.x-q.x,raw.y-q.y),'raw_z':None if raw is None else raw.z})
                self.registrations={k:v for k,v in self.registrations.items() if k>stamp(m)-1.}

    # Drain all feedback under a bounded wall deadline, including when simulated time stalls.
    def wait(self,predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.spin(.03)
            if predicate():return
        raise RuntimeError('condition timed out: '+str(self.status))

    # Validate there is no parallel direct actuation path or truth localization publisher.
    def wiring(self):
        self.wait(lambda:self.samples and self.mcu and self.status,35)
        names=lambda topic,kind:[v.node_name for v in getattr(self.node,kind)(topic)]
        self.wait(lambda:names('/localization/odometry','get_publishers_info_by_topic')==['sentry_estimated_odometry'] and
            'mcu_communicator' in names('/sim/guarded_cmd_vel','get_subscriptions_info_by_topic') and
            names('/sim/serial_cmd_vel','get_publishers_info_by_topic')==['sentry_pty_mcu'],15)
        public=names('/localization/odometry','get_publishers_info_by_topic')
        guarded=names('/sim/guarded_cmd_vel','get_subscriptions_info_by_topic')
        serial=names('/sim/serial_cmd_vel','get_publishers_info_by_topic')
        truth=names('/sim/ground_truth/odometry','get_subscriptions_info_by_topic')
        assert public==['sentry_estimated_odometry'],public
        assert 'mcu_communicator' in guarded and 'sentry_gazebo_bridge' not in guarded,guarded
        assert serial==['sentry_pty_mcu'],serial
        assert not set(truth)&{'sim_ndt','sentry_sim_adapter','sentry_estimated_odometry','sentry_pty_mcu'},truth
        assert self.mcu['endpoint'].startswith('/dev/pts/')
        return {'localization_publishers':public,'guarded_command_subscribers':guarded,
            'serial_actuation_publishers':serial,'truth_consumers':truth,'mcu_endpoint':self.mcu['endpoint'],'mcu_pid':self.mcu['mcu_pid']}

    # Referee-only input must cause a real decision target, movement, measured arrival, and return status.
    def episode(self,name,target):
        self.scenario.publish(String(data='stop'));self.spin(1.)
        self.mode.publish(String(data='auto'));self.spin(.3)
        self.episode_hp=[];self.samples=[];self.errors=[];self.pending.clear();self.trajectories=[];self.arrivals=[];self.goals=[];self.wires=[];self.commands=[]
        began=time.monotonic();self.scenario.publish(String(data=name))
        self.wait(lambda:self.game==4 and self.trajectories and self.hp==(80 if name=='supply' else 321),25)
        self.wait(lambda:bool(self.arrivals) or self.status.get('state')=='stopped',180)
        arrival=self.arrivals[0] if self.arrivals else self.samples[-1]
        episode_hp=list(self.episode_hp)
        self.scenario.publish(String(data='stop'))
        self.wait(lambda:self.game==0 and self.samples[-1]['speed']<.02 and self.status.get('state') in ('stopped','arrived','idle'),5)
        grid=StaticSafetyGrid.load()
        occupied=sum(not grid.points_free([[s['x'],s['y']]]) for s in self.samples)
        frames=[]
        for value in self.wires:
            data=bytes.fromhex(value['last_nav_hex'])
            if len(data)!=21:continue
            assert crc8(data[:8])==data[8] and crc16(data[:-4])==struct.unpack_from('<H',data,17)[0]
            vx,vy,wz=struct.unpack_from('<hhh',data,11)
            if abs(vx)+abs(vy)>0:frames.append([vx/1000,vy/1000,wz/100])
        matched=sum(any(abs(f[0]-c[0])<=.0011 and abs(f[1]-c[1])<=.0011 and abs(f[2]-c[2])<=.011 for c in self.commands) for f in frames)
        result={'scenario':name,'goal':list(target),'wall_duration':time.monotonic()-began,
            'arrival_feedback':bool(self.arrivals),'endpoint_error':math.hypot(arrival['x']-target[0],arrival['y']-target[1]),
            'arrival_actual_speed':arrival['speed'],'occupied_samples':occupied,
            'max_localization_error':max((e['xy'] for e in self.errors),default=math.inf),
            'moving_valid_wire_samples':len(frames),'matching_guarded_commands':matched,
            'decoded_self_hp':sorted(set(episode_hp)),'expected_hp_seen':(80 if name=='supply' else 321) in episode_hp,'decision_goal_seen':any(math.hypot(g[0]-target[0],g[1]-target[1])<.01 for g in self.goals),
            'status_after_game_stop':self.status,'samples':self.samples,'trajectories':self.trajectories,
            'localization_errors':self.errors,'wire_status':self.wires,'guarded_commands':self.commands}
        result['passed']=bool(self.arrivals) and result['endpoint_error']<=.15 and arrival['speed']<.04 and occupied==0 and result['max_localization_error']<.15 and matched>10 and result['decision_goal_seen'] and result['expected_hp_seen']
        return result

def main():
    """Keep partial failure evidence; no test node publishes position, sensor points, trajectories, or drive velocity."""
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--start-supply',action='store_true');a=p.parse_args()
    rclpy.init();qa=FullCheck();records=[];wiring={}
    try:
        wiring=qa.wiring()
        episodes=[('patrol',(-3.219,2.146)),('supply',(-.919,-4.454))]
        for name,target in (episodes[::-1] if a.start_supply else episodes):
            try:result=qa.episode(name,target)
            except Exception as error:
                result={'scenario':name,'passed':False,'error':str(error),'status':qa.status,'mcu':qa.mcu,
                    'samples':qa.samples,'trajectories':qa.trajectories,'wire_status':qa.wires,'localization_errors':qa.errors,'arrivals':qa.arrivals}
            result['wiring']=wiring;records.append(result)
            Path(a.output).write_text(json.dumps(records,indent=2))
            print(json.dumps({k:v for k,v in result.items() if k not in ('samples','trajectories','localization_errors','wire_status','guarded_commands')}),flush=True)
            if not result['passed']:raise RuntimeError('full system acceptance failed')
    finally:
        qa.scenario.publish(String(data='stop'));qa.mode.publish(String(data='stop'));qa.spin(.5)
        Path(a.output).write_text(json.dumps(records,indent=2));qa.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
