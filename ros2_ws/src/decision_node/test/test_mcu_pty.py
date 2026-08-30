"""Black-box PTY regression for the ROS 2 MCU bridge.

The test intentionally uses a real pseudo-terminal and the installed node.  It
therefore covers the serial lifecycle as well as ROS graph wiring, instead of
only exercising the protocol helper in-process.
"""

import os
import pty
import select
import struct
import tempfile
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import UInt16, UInt8


def crc8(data: bytes) -> int:
    # Exact lookup table copied from the deployed HK legacy contract.  This
    # protocol uses a non-standard CRC-8 table, so a generic polynomial helper
    # would test the wrong wire format.
    table = bytes.fromhex(
        "005ebce2613fdd83c29c7e20a3fd1f419dc3217ffca2401e5f01e3bd3e6082dc"
        "237d9fc1421cfea0e1bf5d0380de3c62bee0025cdf81633d7c22c09e1d43a1ff"
        "4618faa427799bc584da3866e5bb5907db856739bae406581947a5fb7826c49a"
        "653bd987045ab8e6a7f91b45c6987a24f8a6441a99c7257b3a6486d85b05e7b9"
        "8cd2306eedb3510f4e10f2ac2f7193cd114fadf3702ecc92d38d6f31b2ec0e50"
        "aff1134dce90722c6d33d18f0c52b0ee326c8ed0530defb1f0ae4c1291cf2d73"
        "ca947628abf517490856b4ea6937d58b5709ebb536688ad495cb2977f4aa4816"
        "e9b7550b88d6346a2b7597c94a14f6a8742ac896154ba9f7b6e80a54d7896b35")
    value = 0xFF
    for byte in data:
        value = table[value ^ byte]
    return value


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8408 if value & 1 else value >> 1
    return value & 0xFFFF


def game_frame() -> bytes:
    header = bytearray(struct.pack("<2sHBBBB", b"HK", 78, 1, 0, 7, 0))
    header.append(crc8(header))
    payload = struct.pack(
        "<4B8H10hBHB2B4H2f3B",
        4, 2, 7, 1,                 # game state
        100, 200, 300, 0, 400, 500, 600, 1,  # red / blue HP
        123, -456, 20, 30, 40, 50, 60, 70, 80, 90,  # enemy positions in cm
        3, 0x1234, 0, 1, 0,         # radar and revival flags
        321, 400, 77, 0,            # robot state
        1.25, -2.5, 1, 0x21, 0,
    )
    frame = bytes(header) + payload
    return frame + struct.pack("<H", crc16(frame)) + b"KH"


def read_available(fd: int, timeout: float = 0.05) -> bytes:
    readable, _, _ = select.select([fd], [], [], timeout)
    return os.read(fd, 4096) if readable else b""


def hk_frames(buffer: bytes):
    frames = []
    offset = 0
    while offset + 4 <= len(buffer):
        start = buffer.find(b"HK", offset)
        if start < 0 or start + 4 > len(buffer):
            break
        length = struct.unpack_from("<H", buffer, start + 2)[0]
        if length not in (21, 78) or start + length > len(buffer):
            offset = start + 1
            continue
        frame = buffer[start:start + length]
        if frame[-2:] == b"KH" and crc8(frame[:8]) == frame[8] and crc16(frame[:-4]) == struct.unpack_from("<H", frame, length - 4)[0]:
            frames.append(frame)
        offset = start + length
    return frames


def generate_test_description():
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    serial_link = os.path.join(tempfile.mkdtemp(prefix="mcu_pty_"), "serial")
    os.symlink(slave_path, serial_link)
    os.set_blocking(master_fd, False)
    return (
        launch.LaunchDescription([
            launch_ros.actions.Node(
                package="decision_node",
                executable="mcu_communicator",
                parameters=[{
                    # A stable symlink lets the test replace the PTY and prove
                    # that the node's automatic reopen path works.
                    "serial_port": serial_link,
                    "baudrate": 115200,
                    "nav_frequency": 100.0,
                    "cmd_vel_timeout": 0.20,
                    "reconnect_interval": 0.05,
                }],
            ),
            launch_testing.actions.ReadyToTest(),
        ]),
        {"serial_master": master_fd, "serial_slave": slave_fd, "serial_link": serial_link},
    )


class TestMcuPty(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("mcu_pty_test_client")
        cls.hp = []
        cls.hero = []
        cls.node.create_subscription(UInt16, "/robot/self_hp", lambda m: cls.hp.append(m.data), 10)
        cls.node.create_subscription(Point, "/enemy/hero_position", lambda m: cls.hero.append((m.x, m.y)), 10)
        cls.cmd = cls.node.create_publisher(Twist, "/cmd_vel", 10)
        cls.motion = cls.node.create_publisher(UInt8, "/motion", 10)
        cls.recover = cls.node.create_publisher(UInt8, "/recover", 10)
        cls.bullet_up = cls.node.create_publisher(UInt8, "/bullet_up", 10)
        cls.bullet_num = cls.node.create_publisher(UInt8, "/bullet_num", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_bidirectional_serial_contract_and_watchdog(self, serial_master, serial_slave, serial_link):
        # Wait for DDS discovery, then feed the 78-byte game packet in fragments.
        self.assertTrue(self.spin_until(lambda: self.cmd.get_subscription_count() == 1))
        frame = game_frame()
        os.write(serial_master, b"noise" + frame[:13])
        os.write(serial_master, frame[13:])
        self.assertTrue(self.spin_until(lambda: self.hp == [321] and self.hero == [(1.23, -4.56)]))

        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = 1.234, -0.5, 2.0
        self.cmd.publish(command)
        for publisher, value in ((self.motion, 2), (self.recover, 2), (self.bullet_up, 1), (self.bullet_num, 5)):
            message = UInt8()
            message.data = value
            publisher.publish(message)
        self.assertTrue(self.spin_until(lambda: self.motion.get_subscription_count() == 1))

        output = bytearray()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            output.extend(read_available(serial_master))
            nav = hk_frames(output)
            has_velocity = any(struct.unpack_from("<hhh", packet, 11) == (1234, -500, 200) and packet[9] == 2 for packet in nav if len(packet) == 21)
            has_motion = any(output[index:index + 7][0:5] == b"\x92\x02\x00\x01\x05" and output[index + 6:index + 7] == b"\xfe" and crc8(output[index:index + 5]) == output[index + 5] for index in range(max(0, len(output) - 6)))
            if has_velocity and has_motion:
                break
        self.assertTrue(has_velocity, "no CRC-valid navigation packet carried cmd_vel and recover")
        self.assertTrue(has_motion, "no CRC-valid motion packet carried motion/bullet command")

        # After the configured watchdog interval, navigation is deliberately zeroed.
        time.sleep(0.30)
        output.clear()
        deadline = time.monotonic() + 1.0
        watchdog_zero = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            output.extend(read_available(serial_master))
            watchdog_zero = any(struct.unpack_from("<hhh", packet, 11) == (0, 0, 0) for packet in hk_frames(output) if len(packet) == 21)
            if watchdog_zero:
                break
        self.assertTrue(watchdog_zero, "stale cmd_vel was not cleared by the watchdog")

        # Closing the PTY makes the node observe a serial error.  Repoint the
        # configured path at a fresh PTY and require a CRC-valid nav packet.
        os.close(serial_master)
        os.close(serial_slave)
        new_master, new_slave = pty.openpty()
        os.set_blocking(new_master, False)
        os.unlink(serial_link)
        os.symlink(os.ttyname(new_slave), serial_link)
        output.clear()
        deadline = time.monotonic() + 2.0
        reconnected = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            output.extend(read_available(new_master))
            reconnected = any(len(packet) == 21 for packet in hk_frames(output))
            if reconnected:
                break
        os.close(new_master)
        os.close(new_slave)
        self.assertTrue(reconnected, "serial reconnect did not resume navigation output")
