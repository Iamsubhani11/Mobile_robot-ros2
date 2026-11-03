#!/usr/bin/env python3
"""Writes the design inspection images into images/. usage: render_all.py robot.urdf images_dir screenshot"""
import sys, os, json, copy
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2_ws', 'src', 'amr_control'))
from render import *
from amr_control.arm_model import POSES, ArmModel
from PIL import Image, ImageDraw, ImageFont
urdf, out, shot = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(out, exist_ok=True)
R = load_robot(urdf); M = ArmModel(open(urdf).read())
yaw = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad_report.json')))['arm_cad_yaw_deg']
cad = {'arm_waist_joint': np.radians(yaw)}
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 26)
def label(img, text):
    d = ImageDraw.Draw(img); d.rectangle((0, 0, img.width, 46), fill=(30, 33, 40)); d.text((14, 8), text, fill=(235, 238, 242), font=font); return img
def save(img, name, text=None):
    if text: label(img, text)
    img.save(os.path.join(out, name)); print('wrote', name, flush=True)
# 1. same angle and same parts as the screenshot: no lidar or camera
bare = copy.copy(R); bare['links'] = {k: ([] if k in ('lidar_mount', 'lidar_link', 'camera_mount', 'camera_link', 'branding', 'gripper_base', 'gripper_pinion', 'gripper_finger_left', 'gripper_finger_right') else v) for k, v in R['links'].items()}
eye, tgt = (0.42, -1.05, 1.42), (0.13, -0.02, 0.10)
mine = render(bare, cad, eye, tgt, size=(1100, 780), fov=25)
s = Image.open(shot).convert('RGB').crop((0, 520, 572, 1050))
s = s.resize((int(s.width * 780 / s.height), 780))
c = Image.new('RGB', (s.width + mine.width, 780)); c.paste(s, (0, 0)); c.paste(mine, (s.width, 0))
d = ImageDraw.Draw(c); d.rectangle((0, 0, c.width, 46), fill=(30, 33, 40)); d.text((14, 8), 'Your screenshot', fill=(235, 238, 242), font=font); d.text((s.width + 14, 8), 'This URDF, same CAD pose (sensors, branding and new gripper hidden)', fill=(235, 238, 242), font=font)
c.save(os.path.join(out, 'cad_comparison.png')); print('wrote compare', flush=True)
# 2. the finished robot
views = [('robot_full.png', 'Finished robot with lidar and camera', (-0.62, -1.20, 0.95), (0.14, 0, 0.14), 25),
         ('robot_front.png', 'Front', (1.8, 0, 0.26), (0, 0, 0.18), 22), ('robot_side.png', 'Side', (0.12, -1.8, 0.2), (0.12, 0, 0.17), 22),
         ('robot_top.png', 'Top', (0.12, -0.001, 1.9), (0.12, 0, 0), 22), ('robot_rear.png', 'Rear', (-1.05, -0.85, 0.7), (-0.02, 0, 0.13), 20),
         ('sensor_mounts.png', 'Lidar and camera on the front of the deck', (0.72, -0.52, 0.48), (0.10, 0, 0.14), 16)]
for f, t, e, g, fov in views:
    save(render(R, {}, e, g, size=(1300, 900), fov=fov), f, t)
# 3. poses
reach = dict(M.solve_ik(0.31, 0, 0.010)); reach['gripper_joint'] = 0.4
poses = [('pose_home.png', 'Pose: home, q = 0 (the CAD pose)', {}), ('pose_stow.png', 'Pose: stow, for driving', POSES['stow']),
         ('pose_floor_pick.png', 'Pose: picking from the floor, 335 mm in front', reach)]
for f, t, q in poses:
    save(render(R, q, (0.95, -1.55, 0.80), (0.12, 0, 0.27), size=(1200, 860), fov=24), f, t)
# 4. gripper
base = M.solve_ik(0.31, 0, 0.010); ims = []
for gv in (0.0, 1.1, 2.1):
    q = dict(base); q['gripper_joint'] = gv
    ims.append(label(render(R, q, (1.0, 0, 0.05), (0.31, 0, 0.06), size=(560, 560), fov=11, grid=False), f'gripper {gv:.1f} rad'))
c = Image.new('RGB', (1680, 560)); [c.paste(im, (i * 560, 0)) for i, im in enumerate(ims)]
c.save(os.path.join(out, 'gripper_travel.png')); print('wrote gripper', flush=True)
# 5. wheel pattern seen from below: the X pattern
wheel = render(R, {}, (0.0, -0.0001, -1.0), (0, 0, 0), size=(1100, 800), fov=22, grid=False)
save(wheel, 'mecanum_wheel_pattern.png', 'From below: the X roller pattern (front is at the top)')
