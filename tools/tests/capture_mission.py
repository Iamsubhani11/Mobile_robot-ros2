#!/usr/bin/env python3
"""Sends a mission and saves frames timed on the mission's own status messages, plus an animated GIF."""
import json, os, sys, time
import numpy as np, rclpy
from PIL import Image as P, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
out, mission = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
rclpy.init(); n = Node('cap', parameter_overrides=[]); img = {}; st = []
def cb(k):
    def f(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        img[k] = (a[..., ::-1] if m.encoding.startswith('bgr') else a)[..., :3].copy()
    return f
for k, t in dict(station='/inspect/station_cam', over='/inspect/overview_cam', drop='/inspect/drop_cam', dbg='/object/debug_image').items():
    n.create_subscription(Image, t, cb(k), qos_profile_sensor_data)
n.create_subscription(String, 'mission/status', lambda m: st.append((time.time(), json.loads(m.data)['status'])), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub = n.create_publisher(String, 'mission/command', 10)
t0 = time.time()
while (len(img) < 4 or pub.get_subscription_count() == 0) and time.time() - t0 < 40: rclpy.spin_once(n, timeout_sec=.1)
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 24)
def save(k, name, title):
    if k not in img: print('no', k); return
    im = P.fromarray(img[k]).convert('RGB'); d = ImageDraw.Draw(im); d.rectangle((0, 0, im.width, 40), fill=(30, 33, 40)); d.text((12, 6), title, fill=(235, 238, 242), font=font)
    im.save(os.path.join(out, name)); print('saved', name, round(time.time() - t0), flush=True)
pub.publish(String(data=mission)); t0 = time.time()
frames = []; last = 0; done = set(); pick_t = None
small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 15)
while time.time() - t0 < 200:
    rclpy.spin_once(n, timeout_sec=.05); now = time.time()
    status = st[-1][1] if st else ''
    if now - last > 0.5:
        last = now
        big = 'station' if any(' PICK ' in ' ' + s[1] + ' ' or s[1].startswith(('ALIGN', 'LOOK')) for s in st[-1:]) else ('drop' if status.startswith('PLACE') else 'over')
        if big in img:
            f = P.fromarray(img[big]).convert('RGB').resize((640, 400)); d = ImageDraw.Draw(f); d.rectangle((0, 0, 640, 24), fill=(30, 33, 40)); d.text((8, 3), status[:70], fill=(235, 238, 242), font=small); frames.append(f)
    if status.startswith('PICK') and pick_t is None: pick_t = now
    if pick_t and now - pick_t > 6.3 and 'pick' not in done:
        done.add('pick'); save('station', 'sim_picking.png', 'Gazebo: fingers down around the block')
    if status.startswith('HOLDING') and 'hold' not in done:
        done.add('hold'); save('station', 'sim_holding_block.png', 'Gazebo: block lifted, arm in the stow pose')
    if status.startswith('NAVIGATE') and 'hold' in done and 'carry' not in done and now - st[-1][0] > 6:
        done.add('carry'); save('over', 'sim_carrying_block.png', 'Gazebo: carrying the block across the room')
    if status.startswith(('DONE', 'FAILED', 'ABORTED')) and now - st[-1][0] > .1 and any(s[1].startswith(('PLACE', 'FAILED')) for s in st):
        rclpy.spin_once(n, timeout_sec=1); save('drop', 'sim_block_placed.png', 'Gazebo: placed in the drop zone'); print(status); break
if frames:
    frames[0].save(os.path.join(out, 'sim_pick_and_place.gif'), save_all=True, append_images=frames[1::1], duration=250, loop=0, optimize=True)
    print('gif frames', len(frames), os.path.getsize(os.path.join(out, 'sim_pick_and_place.gif')) // 1024, 'kB')
print([s[1] for s in st])
