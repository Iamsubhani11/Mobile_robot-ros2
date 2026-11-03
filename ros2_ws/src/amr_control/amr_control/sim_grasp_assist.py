#!/usr/bin/env python3
"""Gazebo only. Joins a block to the gripper when the fingers have closed on it, releases it on open.

Why it exists: Gazebo's DART solver lets objects slide out of a position-controlled gripper.
It refuses to attach unless the block really is at the fingertips, so a missed grasp still
misses, and the robot sees that with its camera. It is not used on the real robot.
"""
import math, re, subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

COLORS = ('red', 'green', 'blue')
MAX_OFFSET = 0.045


def block_position(color):
    out = subprocess.run(['gz', 'model', '-m', f'block_{color}', '-p'], capture_output=True, text=True, timeout=5).stdout
    found = re.findall(r'\[(-?[\d.e-]+) (-?[\d.e-]+) (-?[\d.e-]+)\]', out)
    return [float(v) for v in found[0]] if found else None


class SimGraspAssist(Node):
    def __init__(self):
        super().__init__('sim_grasp_assist')
        self.attach = {c: self.create_publisher(Empty, f'/amr/attach/{c}', 10) for c in COLORS}
        self.detach = {c: self.create_publisher(Empty, f'/amr/detach/{c}', 10) for c in COLORS}
        self.tf_buf = Buffer(); self.tf = TransformListener(self.tf_buf, self)
        self.truth = None; self.held = set(); self.released_at_start = 0
        self.create_subscription(Odometry, 'ground_truth/odom', lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(String, 'gripper/grasp', self.on_grasp, 10)
        # The Gazebo plugin starts attached, so release everything once at startup.
        self.startup = self.create_timer(0.5, self.release_all)

    def release_all(self):
        for c in COLORS:
            self.detach[c].publish(Empty())
        self.released_at_start += 1
        if self.released_at_start >= 12:
            self.destroy_timer(self.startup)
            self.get_logger().info('blocks released, ready')

    def on_grasp(self, msg):
        verb, _, color = msg.data.partition(' ')
        if color not in COLORS:
            return
        if verb == 'detach':
            self.detach[color].publish(Empty()); self.held.discard(color)
            return
        if verb != 'attach' or self.truth is None:
            return
        try:
            t = self.tf_buf.lookup_transform('base_link', 'gripper_tcp', Time()).transform.translation
        except Exception as e:
            self.get_logger().warning(f'no TCP transform: {e}')
            return
        p = self.truth.pose.pose; q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        tcp = (p.position.x + c * t.x - s * t.y, p.position.y + s * t.x + c * t.y, p.position.z + t.z)
        b = block_position(color)
        off = math.dist(tcp, b) if b else float('inf')
        if off > MAX_OFFSET:
            self.get_logger().warning(f'{color} is {off * 1000:.0f} mm from the fingertips: grasp misses')
            return
        self.attach[color].publish(Empty()); self.held.add(color)
        self.get_logger().info(f'holding {color} ({off * 1000:.0f} mm from the fingertip point)')


def main():
    rclpy.init()
    node = SimGraspAssist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
