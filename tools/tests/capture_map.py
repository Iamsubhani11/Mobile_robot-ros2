#!/usr/bin/env python3
"""Saves the live lidar map from a running simulation with nav:=true. usage: capture_map.py out.png"""
import sys, time
import numpy as np, rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
rclpy.init(); n = Node('capture_map'); got = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.__setitem__('g', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t = time.time()
while time.time() - t < 15 and 'g' not in got: rclpy.spin_once(n, timeout_sec=.1)
if 'g' not in got: sys.exit('no costmap received: is the simulation running with nav:=true?')
g = got['g']; a = np.array(g.data).reshape(g.info.height, g.info.width)
img = np.zeros(a.shape + (3,), np.uint8); img[a == -1] = (120, 120, 160)
img[a >= 0] = np.stack([255 - a[a >= 0] * 2.5] * 3, 1).astype(np.uint8)
c = Image.fromarray(img[::-1]).resize((800, 800), Image.NEAREST); d = ImageDraw.Draw(c)
d.rectangle((0, 0, 800, 36), fill=(30, 33, 40))
d.text((10, 5), 'Live map from the lidar (SLAM + costmap)', fill=(235, 238, 242), font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22))
c.save(sys.argv[1]); print('saved', sys.argv[1], a.shape, 'known cells', int((a >= 0).sum()))
