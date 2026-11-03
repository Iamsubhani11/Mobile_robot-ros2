#!/usr/bin/env python3
"""Send a mission:  ros2 run amr_control mission fetch 1.5 0.5 0 red
Also: pick [color], place, go x y yaw, deliver x y yaw, transfer ax ay ayaw bx by byaw [color], cancel"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rclpy.init()
    node = Node('mission_cli')
    pub = node.create_publisher(String, 'mission/command', 10)
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    pub.publish(String(data=' '.join(sys.argv[1:])))
    rclpy.spin_once(node, timeout_sec=0.3)
    node.destroy_node()
    rclpy.try_shutdown()
