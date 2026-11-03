#!/usr/bin/env python3
"""Measures joint limits from the geometry: sweeps each joint and finds where surfaces meet."""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render import *
from scipy.spatial import cKDTree
urdf = sys.argv[1]; R = load_robot(urdf)
rng = np.random.default_rng(0)
def samples(link, spacing=0.0025):
    out = []
    for tris, To, _ in R['links'].get(link, []):
        a = np.linalg.norm(np.cross(tris[:,1]-tris[:,0], tris[:,2]-tris[:,0]), axis=1)/2
        n = np.maximum(np.ceil(a / spacing**2).astype(int), 1)
        idx = np.repeat(np.arange(len(tris)), n)
        u, v = rng.random(len(idx)), rng.random(len(idx)); f = u+v > 1; u[f], v[f] = 1-u[f], 1-v[f]
        t = tris[idx]; p = t[:,0] + u[:,None]*(t[:,1]-t[:,0]) + v[:,None]*(t[:,2]-t[:,0])
        out.append(p @ To[:3,:3].T + To[:3,3])
    return np.concatenate(out) if out else np.zeros((0,3))
P = {l: samples(l) for l in R['links']}
def world(l, poses): return P[l] @ poses[l][:3,:3].T + poses[l][:3,3]
def _pts(links, pose): return np.concatenate([world(l, pose) for l in links])
def contact_free(links_a, links_b, q0, margin=0.006):
    """Keep only points not already within `margin` of the other set in the start pose."""
    pose = fk(R, q0); keep_a, keep_b = {}, {}
    a, b = _pts(links_a, pose), _pts(links_b, pose)
    ta, tb = cKDTree(a), cKDTree(b)
    ka = tb.query(a)[0] > margin; kb = ta.query(b)[0] > margin
    return ka, kb
def sweep(joint, sign, A, B, excl=None, thresh=0.0015, start=0.0, qbase=None):
    q = dict(qbase or {}); q0 = dict(q); ka, kb = contact_free(A, B, q0)
    a = start; last = a
    while abs(a) < 2.8:
        a += sign*np.radians(2); q[joint] = a; pose = fk(R, q)
        pa, pb = _pts(A, pose)[ka], _pts(B, pose)[kb]
        if cKDTree(pb).query(pa)[0].min() < thresh: return last
        last = a
    return last
UP = ['arm_upper']; FO = ['arm_forearm']; TUR = ['arm_turret']
GR = ['arm_wrist','gripper_gear_left','gripper_gear_right','gripper_finger_left','gripper_finger_right','gripper_rocker_left','gripper_rocker_right']
BASE = ['base_link','lidar_link','lidar_mount','camera_link','camera_mount']
out = {}
TUR = ['arm_turret']
ALLA = UP + FO + GR
out['shoulder+ (forward/down), elbow 0'] = sweep('arm_shoulder_joint', 1, ALLA, BASE+TUR)
out['shoulder- (back), elbow 0'] = sweep('arm_shoulder_joint', -1, ALLA, BASE+TUR)
out['shoulder+ , elbow -90'] = sweep('arm_shoulder_joint', 1, ALLA, BASE+TUR, qbase={'arm_elbow_joint':-1.57})
out['shoulder- , elbow -90'] = sweep('arm_shoulder_joint', -1, ALLA, BASE+TUR, qbase={'arm_elbow_joint':-1.57})
out['elbow+ (down)'] = sweep('arm_elbow_joint', 1, FO+GR, UP)
out['elbow- (up/back)'] = sweep('arm_elbow_joint', -1, FO+GR, UP)
out['wrist+ (down/in)'] = sweep('arm_wrist_joint', 1, GR, FO)
out['wrist- (up/out)'] = sweep('arm_wrist_joint', -1, GR, FO)
print(json.dumps({k: round(np.degrees(v),1) for k, v in out.items()}, indent=1))
# Gripper
for s in (1,-1):
    print('gripper sign', s)
    for a in np.radians([0,5,10,15,20,25,30,35,40,45,50,60]):
        q = {'gripper_joint': s*a}
        pose = fk(R, q)
        l, r = world('gripper_finger_left',pose), world('gripper_finger_right',pose)
        print(f"  {np.degrees(a):4.0f} deg  finger min gap {cKDTree(r).query(l)[0].min()*1000:6.1f} mm   mean separation {np.linalg.norm(l.mean(0)-r.mean(0))*1000:6.1f} mm")
