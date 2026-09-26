#!/usr/bin/env python3
"""Run the actual MCU executable against an owned PTY; decoded HK navigation bytes alone actuate Gazebo."""
import json,math,os,pty,signal,struct,subprocess,tempfile,time,tty
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from mcu_wire import crc8,crc16,game_frame

class PtyMcu(Node):
    # A fresh owned /dev/pts endpoint is mandatory; no parameter can select a physical serial device.
    def __init__(self):
        super().__init__('sentry_pty_mcu')
        self.master,self.slave=pty.openpty();tty.setraw(self.slave)
        os.set_blocking(self.master,False)
        path=os.ttyname(self.slave)
        if not path.startswith('/dev/pts/') or not os.isatty(self.slave):raise RuntimeError('not an owned PTY')
        executable=Path(get_package_prefix('decision_node'))/'lib/decision_node/mcu_communicator'
        self.child=subprocess.Popen([str(executable),'--ros-args','-r','__node:=mcu_communicator',
            '-r','/cmd_vel:=/sim/guarded_cmd_vel','-p','use_sim_time:=true','-p','serial_port:='+path,
            '-p','baudrate:=921600','-p','nav_frequency:=50.0','-p','cmd_vel_timeout:=0.5','-p','reconnect_interval:=0.1'],
            stdin=subprocess.DEVNULL)
        self.endpoint=path;self.buffer=bytearray();self.pending_write=bytearray()
        self.scenario='stop';self.nav_count=self.motion_count=self.invalid=self.referee_count=0
        self.last_nav=-math.inf;self.last_referee=self.last_status=self.last_output=0.
        self.command=Twist();self.last_hex='';self.motion=0
        self.output=self.create_publisher(Twist,'/sim/serial_cmd_vel',1)
        self.status=self.create_publisher(String,'/sim/mcu/status',10)
        self.create_subscription(String,'/sim/referee/scenario',self.on_scenario,10)
        self.create_timer(.01,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Actual MCU child '+str(self.child.pid)+' connected only to '+path)

    # Referee scenarios enter only through real serial bytes, never direct ROS referee publishers.
    def on_scenario(self,msg):
        if msg.data not in ('stop','patrol','supply','corrupt_referee'):
            self.get_logger().warning('Unknown referee scenario');return
        self.scenario=msg.data

    # Bound parsing and validate both legacy CRCs before a navigation frame can refresh the actuator.
    def parse(self):
        while self.buffer:
            if self.buffer[:2]==b'HK':
                if len(self.buffer)<9:return
                length=struct.unpack_from('<H',self.buffer,2)[0]
                if length not in (21,78):del self.buffer[0];self.invalid+=1;continue
                if len(self.buffer)<length:return
                frame=bytes(self.buffer[:length]);del self.buffer[:length]
                valid=(frame[-2:]==b'KH' and crc8(frame[:8])==frame[8] and crc16(frame[:-4])==struct.unpack_from('<H',frame,length-4)[0])
                if not valid:self.invalid+=1;continue
                if length==21:
                    vx,vy,wz=struct.unpack_from('<hhh',frame,11)
                    cmd=Twist();cmd.linear.x=vx/1000.;cmd.linear.y=vy/1000.;cmd.angular.z=wz/100.
                    if math.hypot(cmd.linear.x,cmd.linear.y)>.501 or abs(cmd.angular.z)>.501:
                        self.invalid+=1;self.command=Twist();self.last_nav=-math.inf;continue
                    self.command=cmd;self.last_nav=time.monotonic();self.nav_count+=1;self.last_hex=frame.hex()
            elif self.buffer[0]==0x92:
                if len(self.buffer)<7:return
                frame=bytes(self.buffer[:7]);del self.buffer[:7]
                if frame[-1]==0xfe and crc8(frame[:5])==frame[5]:self.motion=frame[1];self.motion_count+=1
                else:self.invalid+=1
            elif len(self.buffer)==1 and self.buffer[0]==ord('H'):return
            else:del self.buffer[0];self.invalid+=1

    # A real serial stall expires decoded velocity by wall time independently of MCU / ROS simulated clocks.
    def tick(self):
        if self.child.poll() is not None:raise RuntimeError('Actual MCU executable exited')
        now=time.monotonic()
        try:
            while True:
                data=os.read(self.master,8192)
                if not data:break
                self.buffer.extend(data)
                if len(self.buffer)>65536:raise RuntimeError('unbounded serial input')
        except BlockingIOError:pass
        self.parse()
        if now-self.last_referee>=.1:
            progress=0 if self.scenario=='stop' else 4
            frame=game_frame(progress,80 if self.scenario=='supply' else 321)
            if self.scenario=='corrupt_referee':frame=frame[:-4]+bytes([frame[-4]^0xff])+frame[-3:]
            self.pending_write.extend(frame);self.last_referee=now;self.referee_count+=1
        if self.pending_write:
            try:
                count=os.write(self.master,self.pending_write);del self.pending_write[:count]
            except BlockingIOError:pass
            if len(self.pending_write)>8192:raise RuntimeError('MCU serial receiver stopped draining')
        if now-self.last_output>=.02:
            self.output.publish(self.command if now-self.last_nav<=.3 else Twist());self.last_output=now
        if now-self.last_status>=.2:
            self.status.publish(String(data=json.dumps({'endpoint':self.endpoint,'mcu_pid':self.child.pid,
                'nav_frames':self.nav_count,'motion_frames':self.motion_count,'invalid_frames':self.invalid,
                'referee_frames_sent':self.referee_count,'scenario':self.scenario,'motion_mode':self.motion,
                'last_nav_hex':self.last_hex,'last_nav_age':max(0.,now-self.last_nav) if math.isfinite(self.last_nav) else None})))
            self.last_status=now

    # Stop only the owned MCU child, close both PTY ends and leave a zero decoded command.
    def close(self):
        if rclpy.ok():self.output.publish(Twist())
        if self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            try:self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:self.child.terminate();self.child.wait(timeout=3)
        os.close(self.master);os.close(self.slave)

def main():
    rclpy.init();node=PtyMcu()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
