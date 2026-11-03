#!/usr/bin/env python3
"""Parallel-jaw rack-and-pinion gripper that replaces the g1, g2.1, link5, link5_2 and grip parts.

It bolts to the two 3 mm holes in wrist part 4.1 and is driven by the existing gripper servo.
All dimensions in mm. Frame F: origin on the servo shaft axis, x along the shaft (towards the
servo body), y across the jaws, z up the gripper, so the fingers point towards -z.
Output: meshes for the URDF in frame F, and print-ready STLs in hardware/gripper_stl.
"""
import math, os, site, sys
sys.path.append(site.getusersitepackages())
import cadquery as cq

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
MESH = os.path.join(ROOT, 'ros2_ws', 'src', 'amr_description', 'meshes', 'gripper')
PRINT = os.path.join(ROOT, 'hardware', 'gripper_stl')

# ---- gear: gap between pads = OPEN_GAP - 2 * PITCH_R * q
MODULE, TEETH, PRESSURE = 1.25, 20, math.radians(20)
PITCH_R = MODULE * TEETH / 2         # 12.5
ADD, DED = MODULE, 1.25 * MODULE
CIRC = math.pi * MODULE
BACKLASH = 0.10
OPEN_GAP, Q_MAX = 56.0, 2.2

# ---- layers along x, measured from wrist part 4.1
RIM_BACK = 5.9                       # main face of 4.1
FLOOR_BACK, FLOOR_FRONT = 3.0, 0.6   # clears the 3.1 pin pad on 4.1
GEAR_BACK, GEAR_FRONT = 0.4, -7.6
RACK_BACK, RACK_FRONT = 0.3, -7.6
COVER_BACK, COVER_FRONT = -7.9, -10.9
FINGER_BACK, FINGER_FRONT = -11.2, -17.9
HUB_BACK, HUB_R = 2.3, 10.5          # sits on the servo horn, inside the 20 mm bore of 4.1
TIP_BACK, TIP_FRONT = -7.0, -22.0    # the wide pad end of each finger
TCP_X = (TIP_BACK + TIP_FRONT) / 2

# ---- layout across and along the gripper
Y_END, Z_TOP = 49.0, 21.9            # the wrist bar of 4.1 starts at z = 22
RACK_ROOT = PITCH_R + DED            # rack body starts here
RACK_TOP, CHANNEL_TOP = 20.5, 20.7
GUIDE_TOP = PITCH_R - ADD - 0.25     # filler block that the tooth tips slide over
RACK_Y0, RACK_Y1 = -6.0, 48.0
BOLT_Z, BOLT_Y = (RACK_ROOT + RACK_TOP) / 2, (35.0, 44.0)
PAD_T, PAD_H = 2.5, 26.0
TIP_Z = -94.0   # at the home pose the fingertips stay above the lidar scan plane
FINGER_IN = OPEN_GAP / 2 + PAD_T     # finger inner face at q = 0
MOUNT = ((-7.97, -18.14), (11.95, -18.15))   # holes in part 4.1
BOSS = (-13.4, 32.4, 13.4)           # raised boss on 4.1, keys the base against rotation
SCREWS = ((-42, 0), (-24, 0), (24, 0), (42, 0))

def yz(x):
    """Sketch plane at x: u = y, v = z, extrudes towards +x."""
    return cq.Workplane(cq.Plane(origin=(x, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0)))

def prism(pts, x0, x1):
    return yz(x0).polyline(pts).close().extrude(x1 - x0)

def box(x0, x1, y0, y1, z0, z1):
    return prism([(y0, z0), (y1, z0), (y1, z1), (y0, z1)], x0, x1)

def holes(solid, pts, d, x0, x1):
    return solid.cut(yz(x0).pushPoints(list(pts)).circle(d / 2).extrude(x1 - x0))

def involute_tooth(center):
    rb, ra, rf = PITCH_R * math.cos(PRESSURE), PITCH_R + ADD, PITCH_R - DED
    inv = lambda a: math.tan(a) - a
    half = (CIRC / 2 - BACKLASH / 2) / (2 * PITCH_R) + inv(PRESSURE)   # half tooth angle at the base circle
    left, right = [], []
    for i in range(7):
        r = max(rb, rf) + (ra - max(rb, rf)) * i / 6
        t = half - inv(math.acos(min(1.0, rb / r)))
        left.append((r, center + t)); right.append((r, center - t))
    return [(rf, center + half)] + left + list(reversed(right)) + [(rf, center - half)]

def pinion():
    pts = []
    for k in range(TEETH):      # a tooth is centred on +z at q = 0
        pts += involute_tooth(math.pi / 2 - 2 * math.pi * k / TEETH)
    gear = prism([(r * math.cos(a), r * math.sin(a)) for r, a in pts], GEAR_FRONT, GEAR_BACK)
    gear = gear.union(yz(GEAR_BACK).circle(HUB_R).extrude(HUB_BACK - GEAR_BACK))
    gear = holes(gear, [(0, 0)], 3.4, GEAR_FRONT, HUB_BACK)                       # horn screw into the shaft
    gear = holes(gear, [(0, 0)], 7.0, GEAR_FRONT, GEAR_BACK - 3.0)                # screw head recess
    horn = [(7 * math.cos(a), 7 * math.sin(a)) for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]
    return holes(gear, horn, 1.8, GEAR_FRONT, HUB_BACK)                           # M2 screws into a 14 mm circle horn

def rack():
    """The upper rack, on the -y side so its jaw stays clear of the forearm. Rotated 180 degrees about x
    it is the lower rack, so print two."""
    pts = [(RACK_Y0, RACK_TOP), (RACK_Y1, RACK_TOP), (RACK_Y1, RACK_ROOT)]
    half_pitch = CIRC / 4 - BACKLASH / 4; slope = math.tan(PRESSURE)
    root_half, tip_half = half_pitch + DED * slope, half_pitch - ADD * slope
    k = math.ceil(RACK_Y1 / CIRC)
    while (k - 0.5) * CIRC > RACK_Y0 + root_half + 0.5:
        c = (k - 0.5) * CIRC          # a gap is on y = 0, because a pinion tooth is on +z at q = 0
        if c + root_half < RACK_Y1 - 0.5:
            pts += [(c + root_half, RACK_ROOT), (c + tip_half, PITCH_R - ADD + 0.1), (c - tip_half, PITCH_R - ADD + 0.1), (c - root_half, RACK_ROOT)]
        k -= 1
    pts.append((RACK_Y0, RACK_ROOT))
    pts = [(-y, z) for y, z in pts]
    return holes(prism(pts, RACK_FRONT, RACK_BACK), [(-y, BOLT_Z) for y in BOLT_Y], 2.6, RACK_FRONT, RACK_BACK - 1.5)

def base():
    b = box(COVER_BACK, FLOOR_BACK, -Y_END, Y_END, -Z_TOP, Z_TOP)
    b = b.union(box(FLOOR_BACK, RIM_BACK, -20.0, 35.5, -Z_TOP, Z_TOP))
    b = b.cut(box(FLOOR_BACK, RIM_BACK + 1, BOSS[0], BOSS[1], -BOSS[2], BOSS[2]))
    # rack channels, open at the far end so the racks slide in
    b = b.cut(box(COVER_BACK - 1, FLOOR_FRONT, -Y_END - 1, 36.0, GUIDE_TOP, CHANNEL_TOP))
    b = b.cut(box(COVER_BACK - 1, FLOOR_FRONT, -36.0, Y_END + 1, -CHANNEL_TOP, -GUIDE_TOP))
    b = b.cut(yz(COVER_BACK - 1).circle(PITCH_R + ADD + 1.2).extrude(FLOOR_FRONT - COVER_BACK + 1))
    b = holes(b, [(0, 0)], 22.0, FLOOR_FRONT - 0.1, FLOOR_BACK + 0.1)
    b = holes(b, MOUNT, 3.3, FLOOR_FRONT - 0.1, RIM_BACK + 0.1)
    b = holes(b, MOUNT, 5.8, FLOOR_FRONT - 0.1, FLOOR_FRONT + 2.0)
    return holes(b, SCREWS, 2.6, COVER_BACK - 0.1, COVER_BACK + 7.0)

def cover():
    c = box(COVER_FRONT, COVER_BACK, -Y_END, Y_END, -Z_TOP, Z_TOP)
    c = c.cut(box(COVER_FRONT - 1, COVER_BACK + 1, -47.0, -4.0, BOLT_Z - 1.9, BOLT_Z + 1.9))
    c = c.cut(box(COVER_FRONT - 1, COVER_BACK + 1, 4.0, 47.0, -BOLT_Z - 1.9, -BOLT_Z + 1.9))
    # the forearm swings through here when the wrist folds up
    c = c.cut(box(COVER_FRONT - 1, COVER_BACK + 1, -2.0, 30.0, 5.0, Z_TOP + 1))
    c = holes(c, SCREWS, 3.4, COVER_FRONT - 0.1, COVER_BACK + 0.1)
    return holes(c, [(0, 0)], 8.0, COVER_FRONT - 0.1, COVER_BACK + 0.1)

def finger(side):
    """side = +1 left (+y), hangs from the lower rack; side = -1 right, from the upper rack."""
    i, t = FINGER_IN, TIP_Z
    if side < 0:
        pts = [(i, BOLT_Z + 5.2), (48, BOLT_Z + 5.2), (48, BOLT_Z - 5.8), (42, -8), (40, t + 34), (i + 8, t + 6), (i + 5, t), (i, t)]
    else:
        pts = [(i, -BOLT_Z + 5.8), (48, -BOLT_Z + 5.8), (48, -BOLT_Z - 5.2), (42, -BOLT_Z - 14), (40, t + 34), (i + 8, t + 6), (i + 5, t), (i, t)]
    f = prism([(side * y, z) for y, z in pts], FINGER_FRONT, FINGER_BACK)
    tip = box(TIP_FRONT, TIP_BACK, min(side * i, side * (i + 5)), max(side * i, side * (i + 5)), t, t + PAD_H)
    f = f.union(tip)
    return holes(f, [(side * y, -side * BOLT_Z) for y in BOLT_Y], 3.4, FINGER_FRONT, FINGER_BACK)

def pad(side):
    """Replaceable grip pad, shallow grid face. Print in TPU, or use it as a mould for silicone."""
    y0 = OPEN_GAP / 2
    p = box(TIP_FRONT, TIP_BACK, min(side * y0, side * (y0 + PAD_T)), max(side * y0, side * (y0 + PAD_T)), TIP_Z, TIP_Z + PAD_H)
    y_face = side * y0
    groove_y = (min(y_face, y_face + side * 0.6), max(y_face, y_face + side * 0.6))
    for k in range(1, 5):
        x = TIP_FRONT + 3.0 * k
        p = p.cut(box(x - 0.4, x + 0.4, groove_y[0], groove_y[1], TIP_Z - 1, TIP_Z + PAD_H + 1))
    for k in range(1, 9):
        z = TIP_Z + 2.9 * k
        p = p.cut(box(TIP_FRONT - 1, TIP_BACK + 1, groove_y[0], groove_y[1], z - 0.4, z + 0.4))
    return p

def parts():
    return {'pinion': pinion(), 'rack_upper': rack(), 'base': base(), 'cover': cover(),
            'finger_left': finger(1), 'finger_right': finger(-1), 'pad_left': pad(1), 'pad_right': pad(-1)}

def flat(part, flip=False):
    """Lay a part on the print bed: thickness (x) becomes z."""
    p = part.rotate((0, 0, 0), (0, 1, 0), -90 if not flip else 90)
    b = p.val().BoundingBox()
    return p.translate((-(b.xmin + b.xmax) / 2, -(b.ymin + b.ymax) / 2, -b.zmin))

if __name__ == '__main__':
    os.makedirs(MESH, exist_ok=True); os.makedirs(PRINT, exist_ok=True)
    P = parts()
    P['rack_lower'] = P['rack_upper'].rotate((0, 0, 0), (1, 0, 0), 180)
    for name, part in P.items():
        cq.exporters.export(part, os.path.join(MESH, name + '.stl'), tolerance=0.02, angularTolerance=0.15)
    prints = {'pinion': P['pinion'], 'rack': P['rack_upper'], 'base': P['base'], 'cover': P['cover'],
              'finger_left': P['finger_left'], 'finger_right': P['finger_right'], 'pad_left_tpu': P['pad_left'], 'pad_right_tpu': P['pad_right']}
    for name, part in prints.items():
        out = flat(part)
        cq.exporters.export(out, os.path.join(PRINT, name + '.stl'), tolerance=0.02, angularTolerance=0.15)
        b = out.val().BoundingBox()
        print(f'{name:22s} {b.xlen:6.1f} x {b.ylen:6.1f} x {b.zlen:5.1f} mm  {part.val().Volume()/1000:5.1f} cm3  valid={part.val().isValid()} solids={len(part.solids().vals())}')
