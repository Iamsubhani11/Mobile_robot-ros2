#!/usr/bin/env python3
"""grab.py out_dir name topic [topic...]: saves one frame from each sensor_msgs/Image topic."""
import sys, os, time
import numpy as np, rclpy
from PIL import Image as P
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
out, name, topics = sys.argv[1], sys.argv[2], sys.argv[3:]
rclpy.init(); n = Node('grab'); got = {}
def cb(t):
    def f(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding.startswith('bgr'): a = a[..., ::-1]
        got[t] = a[..., :3]
    return f
for t in topics: n.create_subscription(Image, t, cb(t), qos_profile_sensor_data)
t0 = time.time()
while len(got) < len(topics) and time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.1)
for t, a in got.items():
    P.fromarray(a).save(os.path.join(out, f"{name}_{t.strip('/').replace('/', '_')}.png"))
print('saved', list(got), 'missing', [t for t in topics if t not in got])
