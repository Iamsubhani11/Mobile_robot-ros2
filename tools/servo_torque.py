#!/usr/bin/env python3
"""Static gravity torque on each arm joint over the whole joint range, from URDF masses.
usage: servo_torque.py robot.urdf [payload_kg]"""
import sys, os, xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render import *
urdf = sys.argv[1]; payload = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
R = load_robot(urdf); root = ET.parse(urdf).getroot()
mass = {}
for l in root.findall('link'):
    i = l.find('inertial')
    if i is not None:
        o = i.find('origin'); xyz = [float(v) for v in o.get('xyz').split()] if o is not None else [0, 0, 0]
        mass[l.get('name')] = (float(i.find('mass').get('value')), np.array(xyz))
J = R['joints']
CHAIN = {'arm_shoulder_joint': 'arm_upper', 'arm_elbow_joint': 'arm_forearm', 'arm_wrist_joint': 'arm_wrist'}
def after(link):
    out = {link}; ch = True
    while ch:
        ch = False
        for c, j in J.items():
            if j['parent'] in out and c not in out: out.add(c); ch = True
    return out
def torque(q, joint):
    link = CHAIN[joint]; p = fk(R, q); T = p[link]; o = T[:3, 3]; ax = T[:3, :3] @ J[link]['axis']; t = 0.0
    for l in after(link):
        if l in mass:
            m, c = mass[l]; w = p[l][:3, :3] @ c + p[l][:3, 3]
            t += np.dot(np.cross(w - o, np.array([0, 0, -9.81 * m])), ax)
    if payload:
        w = p['gripper_tcp'][:3, 3]; t += np.dot(np.cross(w - o, np.array([0, 0, -9.81 * payload])), ax)
    return t
lim = {j: J[CHAIN[j]]['lim'] for j in CHAIN}
worst = {j: (0, None) for j in CHAIN}
for s in np.linspace(*lim['arm_shoulder_joint'], 21):
    for e in np.linspace(*lim['arm_elbow_joint'], 21):
        for w in np.linspace(*lim['arm_wrist_joint'], 11):
            q = {'arm_shoulder_joint': s, 'arm_elbow_joint': e, 'arm_wrist_joint': w}
            for j in CHAIN:
                t = abs(torque(q, j))
                if t > worst[j][0]: worst[j] = (t, (round(s, 2), round(e, 2), round(w, 2)))
named = {'CAD pose (q=0)': {}, 'grasp, gripper on floor': {'arm_shoulder_joint': .264, 'arm_elbow_joint': .229, 'arm_wrist_joint': -.005},
         'hover over grasp spot': {'arm_shoulder_joint': .045, 'arm_elbow_joint': .089, 'arm_wrist_joint': .354},
         'stow': {'arm_shoulder_joint': -.95, 'arm_elbow_joint': -.6, 'arm_wrist_joint': .9}}
print(f'payload {payload*1000:.0f} g. N*m (kg*cm)')
for name, q in named.items():
    print(f'{name:26s}', '  '.join(f"{j[4:-6]} {abs(torque(q, j)):.2f} ({abs(torque(q, j))*10.2:.0f})" for j in CHAIN))
for j, (t, q) in worst.items():
    print(f'worst {j[4:-6]:9s} {t:.2f} N*m = {t*10.2:.1f} kg*cm at s,e,w={q}')
