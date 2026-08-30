"""Behavior-tree regression through the public ROS 2 decision-node interface."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, Int32, UInt16, UInt8


def generate_test_description():
    parameters = {
        "tick_hz": 40,
        # Keep the central-occupy latch out of these independent behavior tests.
        # Its ROS1-compatible threshold is separately fixed in decision.yaml.
        "central_threshold": 999,
        "goals.occupy.point_0.x": 1.0, "goals.occupy.point_0.y": 1.0,
        "goals.occupy.point_1.x": 2.0, "goals.occupy.point_1.y": 2.0,
        "goals.occupy.point_2.x": 3.0, "goals.occupy.point_2.y": 3.0,
        "goals.occupy.point_3.x": 4.0, "goals.occupy.point_3.y": 4.0,
        "goals.supply.x": 6.0, "goals.supply.y": 6.0,
        "goals.waitforop.x": 0.5, "goals.waitforop.y": 0.5,
        "goals.retreat.x": 9.0, "goals.retreat.y": 9.0,
        "goals.radical.x": 5.0, "goals.radical.y": 5.0,
    }
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="decision_node", executable="strategy_node", parameters=[parameters]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestStrategyScenarios(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("strategy_scenarios_client")
        cls.goals, cls.motion, cls.recover, cls.bullet_up, cls.bullet_num = [], [], [], [], []
        cls.node.create_subscription(PointStamped, "/clicked_point", lambda m: cls.goals.append((m.point.x, m.point.y)), 10)
        cls.node.create_subscription(UInt8, "/motion", lambda m: cls.motion.append(m.data), 10)
        cls.node.create_subscription(UInt8, "/recover", lambda m: cls.recover.append(m.data), 10)
        cls.node.create_subscription(UInt8, "/bullet_up", lambda m: cls.bullet_up.append(m.data), 10)
        cls.node.create_subscription(UInt8, "/bullet_num", lambda m: cls.bullet_num.append(m.data), 10)
        cls.game = cls.node.create_publisher(UInt8, "/referee/game_progress", 10)
        cls.hp = cls.node.create_publisher(UInt16, "/referee/remain_hp", 10)
        cls.bullets = cls.node.create_publisher(UInt16, "/referee/bullet_remain", 10)
        cls.arrived = cls.node.create_publisher(Bool, "/dstar_status", 10)
        cls.friendly = cls.node.create_publisher(Int32, "/referee/friendly_score", 10)
        cls.enemy = cls.node.create_publisher(Int32, "/referee/enemy_score", 10)
        cls.target = cls.node.create_publisher(UInt8, "/radar/suggested_target", 10)
        cls.hero = cls.node.create_publisher(Point, "/enemy/hero_position", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @staticmethod
    def publish(publisher, message_type, value):
        message = message_type()
        if isinstance(value, tuple):
            message.x, message.y = value
        else:
            message.data = value
        publisher.publish(message)

    def test_push_supply_radical_and_harm_retreat(self):
        # Discover all inputs before the state sequence begins.
        self.assertTrue(self.spin_until(lambda: self.game.get_subscription_count() == 1))
        for publisher, kind, value in (
            (self.hp, UInt16, 400), (self.bullets, UInt16, 50), (self.game, UInt8, 4),
        ):
            self.publish(publisher, kind, value)
        self.assertTrue(self.spin_until(lambda: (1.0, 1.0) in self.goals and 3 in self.motion), "game start did not produce INITPUSH output")

        # The real arrival feedback moves PUSH/INITPUSH into OCCUPY, whose
        # normal, non-attacked motion mode is 1.
        self.publish(self.arrived, Bool, True)
        self.assertTrue(self.spin_until(lambda: 1 in self.motion), "arrival did not move INITPUSH into OCCUPY")

        # Low HP plus arrival enters supply, asks for free recovery and buys the delta to max bullet.
        for publisher, kind, value in (
            (self.hp, UInt16, 50), (self.arrived, Bool, True),
        ):
            self.publish(publisher, kind, value)
        self.assertTrue(self.spin_until(lambda: (6.0, 6.0) in self.goals and 1 in self.recover and 1 in self.bullet_up and 100 in self.bullet_num), "supply output disagrees with ROS1 behavior")

        # Once safe, a sufficient score advantage and sufficient bullets select the radar target.
        for publisher, kind, value in (
            (self.arrived, Bool, False), (self.hp, UInt16, 400), (self.friendly, Int32, 100),
            (self.enemy, Int32, 0), (self.target, UInt8, 0), (self.hero, Point, (12.3, -4.5)),
        ):
            self.publish(publisher, kind, value)
        self.assertTrue(self.spin_until(lambda: (12.3, -4.5) in self.goals and 1 in self.motion), "radical chase did not follow the suggested target")

        # Remove the advantage to fall back to WAITFOROP, then induce >50 damage in 2 s.
        for publisher, kind, value in ((self.friendly, Int32, 0), (self.enemy, Int32, 0)):
            self.publish(publisher, kind, value)
        self.assertTrue(self.spin_until(lambda: (0.5, 0.5) in self.goals), "neutral fallback did not choose WAITFOROP")
        self.publish(self.hp, UInt16, 330)
        self.assertTrue(self.spin_until(lambda: (9.0, 9.0) in self.goals), "two-second intense-harm latch did not choose retreat")

        # Death has priority over all normal actions; once HP returns, the
        # persisted RESPAWN state transitions to SUPPLY.
        motion_three_before_respawn = self.motion.count(3)
        self.publish(self.hp, UInt16, 0)
        self.assertTrue(self.spin_until(lambda: 0 in self.motion), "dead sentry did not enter RESPAWN")
        self.publish(self.hp, UInt16, 400)
        self.assertTrue(self.spin_until(lambda: self.motion.count(3) > motion_three_before_respawn), "revived sentry did not transition from RESPAWN to SUPPLY")
