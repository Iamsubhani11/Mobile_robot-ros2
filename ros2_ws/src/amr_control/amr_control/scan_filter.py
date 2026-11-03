#!/usr/bin/env python3
"""scan_raw -> scan: removes returns that land on the robot itself.

The lidar sits in front of the arm pedestal, so about 70 degrees behind it hit the robot. Left in,
SLAM maps the robot and Nav2 sees its own centre as an obstacle and refuses to plan.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        for p, d in (('lidar_x', 0.100), ('half_x', 0.170), ('half_y', 0.165), ('min_range', 0.12)):
            self.declare_parameter(p, d)
        self.lx = self.get_parameter('lidar_x').value
        self.hx = self.get_parameter('half_x').value; self.hy = self.get_parameter('half_y').value
        self.rmin = self.get_parameter('min_range').value
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        out = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.rmin:
                out.append(math.inf)
                continue
            a = msg.angle_min + i * msg.angle_increment
            x, y = self.lx + r * math.cos(a), r * math.sin(a)
            out.append(math.inf if abs(x) < self.hx and abs(y) < self.hy else r)
        msg.ranges = out
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
