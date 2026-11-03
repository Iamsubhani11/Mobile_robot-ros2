#!/usr/bin/env python3
"""Compares the finished URDF (its meshes placed by its joints) against the exact solids in the STEP file.
usage: verify_dimensions.py robot.urdf report.md"""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render import *
urdf, out = sys.argv[1], sys.argv[2]
rep = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cad_report.json')))
R = load_robot(urdf); q = {'arm_waist_joint': np.radians(rep['arm_cad_yaw_deg'])}
pose = fk(R, q)
def box(links):
    pts = []
    for l in links:
        for tris, To, _ in R['links'][l]:
            T = pose[l] @ To; pts.append(tris.reshape(-1, 3) @ T[:3, :3].T + T[:3, 3])
    p = np.concatenate(pts); return p.min(0), p.max(0)
rows = []; worst = 0.0
SENSORS = {'lidar_mount', 'lidar_link', 'camera_mount', 'camera_link', 'branding', 'gripper_base', 'gripper_pinion', 'gripper_finger_left', 'gripper_finger_right'}
ALL = [l for l in R['links'] if R['links'][l] and l not in SENSORS]
pairs = [('Whole robot (without lidar, camera, branding and gripper)', ALL, 'whole_assembly'), ('Chassis + motors + arm pedestal (base_link)', ['base_link'], 'base_link'),
         ('Arm turret (basecap + shoulder servo)', ['arm_turret'], 'arm_turret'), ('Upper arm (link1.1)', ['arm_upper'], 'arm_upper'),
         ('Forearm (link2.1 + link3.1 + 2 servos)', ['arm_forearm'], 'arm_forearm'), ('Wrist (4.1 + gripper servo)', ['arm_wrist'], 'arm_wrist')]
for w in ('front_left', 'front_right', 'rear_left', 'rear_right'):
    pairs.append((f'Wheel {w}', [w + '_wheel'], 'wheel_' + w))
for name, links, key in pairs:
    lo, hi = box(links); clo, chi = np.array(rep['cad_boxes'][key][0]), np.array(rep['cad_boxes'][key][1])
    size, csize = (hi - lo) * 1000, (chi - clo) * 1000; dpos = max(np.abs(lo - clo).max(), np.abs(hi - chi).max()) * 1000
    dsize = np.abs(size - csize).max(); worst = max(worst, dsize)
    rows.append((name, csize, size, dsize, dpos))
lines = ['# Dimension check: URDF against your CAD', '',
         'CAD column: exact solids in `cad/mobile_base_1_robotic_arm.STEP`. URDF column: the finished robot\'s meshes placed by its joints, with the waist turned to the CAD pose.',
         'Regenerate: `python3 tools/verify_dimensions.py robot.urdf docs/DIMENSIONS.md`.', '',
         '| Part | CAD size x × y × z (mm) | URDF size (mm) | Size difference (mm) | Position difference (mm) |', '|---|---|---|---|---|']
for name, c, u, ds, dp in rows:
    lines.append(f"| {name} | {c[0]:.2f} × {c[1]:.2f} × {c[2]:.2f} | {u[0]:.2f} × {u[1]:.2f} × {u[2]:.2f} | {ds:.2f} | {dp:.2f} |")
lines += ['', f'Largest size difference: **{worst:.2f} mm**.', '', 'Key measurements read from the CAD:', '',
          f"- Chassis: 295 × 215 mm, deck {rep['deck_z_m']*1000:.1f} mm above the ground, {rep['ground_clearance_m']*1000:.1f} mm ground clearance",
          f"- Wheels: {rep['wheel_radius_m']*2000:.1f} mm diameter, axles at x = ±82.5 mm, wheel centres at y = ±131.9 mm",
          f"- Arm: waist at z = {rep['joints_m']['waist'][2]*1000:.1f} mm, shoulder to elbow {np.hypot(*np.array(rep['joints_m']['elbow'])[[0,2]] - np.array(rep['joints_m']['shoulder'])[[0,2]])*1000:.1f} mm, elbow to wrist {np.hypot(*np.array(rep['joints_m']['wrist'])[[0,2]] - np.array(rep['joints_m']['elbow'])[[0,2]])*1000:.1f} mm",
          '', 'The STL files are tessellated at 0.25 mm, so a few hundredths of a millimetre of difference is expected. FL and RR wheels are mirrored and each wheel is centred on its bushing axis (the CAD has two of them 0.2 to 0.4 mm off): see `docs/CAD_FINDINGS.md`.']
open(out, 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines[6:6 + len(rows)])); print('worst', round(worst, 3))
