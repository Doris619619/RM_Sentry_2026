#!/usr/bin/env python3
"""Terminal keyboard with short-lived key events and zero on every normal exit."""
import fcntl
import os
import select
import signal
import sys
import termios
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

KEYS = {'w': (0.2, 0., 0.), 's': (-0.2, 0., 0.),
        'a': (0., 0.2, 0.), 'd': (0., -0.2, 0.),
        'q': (0., 0., 0.3), 'e': (0., 0., -0.3)}

def main():
    if not sys.stdin.isatty():
        raise SystemExit('请在 Ubuntu 交互终端运行键盘控制。')
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    runtime = os.environ.get('XDG_RUNTIME_DIR', '/tmp')
    lock_file = open(f'{runtime}/sentry-keyboard-{os.getuid()}-{domain}.lock', 'w')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('此 ROS domain 已有键盘控制程序，请先退出旧程序。')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sentry_keyboard')
    node.declare_parameter('use_sim_time', True) if not node.has_parameter('use_sim_time') else None
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    old = termios.tcgetattr(sys.stdin)
    def stop_handler(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGINT, stop_handler)
    print('W/S 前后 | A/D 横移 | Q/E 旋转 | 空格 停止 | Esc 退出\n'
          '按住方向键连续运动；松键约 0.3 秒停止（终端按键重复，首次重复可能有短暂停顿）。', flush=True)
    cmd = Twist()
    last_key = -1e10
    connected = False
    disconnected = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if not connected and pub.get_subscription_count() > 0:
                print('控制接口已连接。', flush=True)
                connected = True
            if select.select([sys.stdin], [], [], 0.02)[0]:
                raw_key = os.read(sys.stdin.fileno(), 1)
                if not raw_key:  # Terminal disconnected: stop, never spin on EOF.
                    disconnected = True
                    break
                key = raw_key.decode('ascii', errors='ignore').lower()
                if key in ('\x1b', '\x03'):
                    break
                cmd = Twist()
                if key in KEYS:
                    cmd.linear.x, cmd.linear.y, cmd.angular.z = KEYS[key]
                last_key = time.monotonic()
            if time.monotonic() - last_key > 0.3:
                cmd = Twist()
            pub.publish(cmd)
    except KeyboardInterrupt:
        pass
    except OSError:
        disconnected = True
    finally:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        except termios.error:
            disconnected = True  # A disconnected PTY cannot restore attributes.
        if rclpy.ok():
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock_file.close()
        try:
            if not disconnected:
                print('\n已发送停止指令。', flush=True)
        except OSError:
            pass

if __name__ == '__main__':
    main()
