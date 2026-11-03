#!/usr/bin/env python3
"""The single arbiter for robot modes: MANUAL, AUTO, STOP.

Only this node publishes /cmd_vel, so a teleop and Nav2 can never fight over the wheels.
STOP latches: it holds zero velocity until someone selects another mode.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String

MODES = ('MANUAL', 'AUTO', 'STOP')


class ModeMux(Node):
    def __init__(self):
        super().__init__('mode_mux')
        self.mode = 'MANUAL'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, 'mode', latched)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, 'mode_cmd', self.on_mode, 10)
        self.create_subscription(Twist, 'cmd_vel_teleop', lambda m: self.forward(m, 'MANUAL'), 10)
        self.create_subscription(Twist, 'cmd_vel_nav', lambda m: self.forward(m, 'AUTO'), 10)
        self.create_subscription(Twist, 'cmd_vel_auto', lambda m: self.forward(m, 'AUTO'), 10)
        self.create_timer(0.1, self.tick)
        self.mode_pub.publish(String(data=self.mode))

    def on_mode(self, msg):
        m = msg.data.strip().upper()
        if m in MODES and m != self.mode:
            self.get_logger().info(f'mode {self.mode} -> {m}')
            self.mode = m
            self.cmd_pub.publish(Twist())
            self.mode_pub.publish(String(data=m))

    def forward(self, msg, source_mode):
        if self.mode == source_mode:
            self.cmd_pub.publish(msg)

    def tick(self):
        if self.mode == 'STOP':
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ModeMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
