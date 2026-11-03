#!/usr/bin/env python3
"""Generate the robot's meshes, inertias and kinematic tree from the SolidWorks STEP export.

Input : cad/mobile_base_1_robotic_arm.STEP  (the assembly shown in the project screenshot)
Output: ros2_ws/src/amr_description/meshes/{visual,collision}/*.stl
        ros2_ws/src/amr_description/urdf/generated/amr_cad.xacro
        tools/cad_report.json

Nothing about the geometry is typed in by hand: every joint axis is read from the
cylindrical bore/servo-shaft faces that two neighbouring parts share in the CAD.
Assumptions that CAD cannot answer (densities, motor/battery masses, joint limits)
are all in the CONFIG section below. Edit them and re-run this script.
"""
import json, math, os, site, sys
sys.path.append(site.getusersitepackages())
import numpy as np
from scipy.spatial import ConvexHull
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Trsf

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
STEP = os.path.join(ROOT, 'cad', 'mobile_base_1_robotic_arm.STEP')
PKG = os.path.join(ROOT, 'ros2_ws', 'src', 'amr_description')

# ===================================== CONFIG ========================================
PRINT_DENSITY = 0.75e-6      # kg/mm^3. PLA 1.24 g/cm3 at roughly 60 % effective fill. WEIGH YOUR PARTS.
FIXED_MASS = {               # kg, real parts whose CAD volume does not reflect their mass
    'orange_johnson_motor': 0.25,   # 36 mm Johnson gearmotor (typical)
    'Servo': 0.055,                 # MG996R
    'bush_m_w2': 0.015,
}
WHEEL_MASS = 0.09            # kg, one 48 mm mecanum wheel
PART_DENSITY = {'pbc1': 1.85e-6}   # the plate modelled as a PCB/tray
# Parts that are not in the CAD but will be inside the chassis (kg, x y z in metres from
# base_link, box size). Battery goes at the rear to counterbalance the arm.
EXTRA_BODIES = [
    ('battery_3s', 0.45, (-0.075, 0.0, 0.075), (0.135, 0.045, 0.030)),
    ('electronics', 0.25, (0.060, 0.0, 0.075), (0.100, 0.100, 0.030)),
]
# Joint limits in radians around the CAD pose (q=0), from tools/sweep_limits.py.
LIMITS = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joint_limits.json')))
SERVO_EFFORT = 1.0           # N*m, MG996R near stall at 6 V
# tools/servo_torque.py: the shoulder needs 1.28 N*m and the elbow 0.67 N*m just to hold still, so
# an MG996R there stalls (Gazebo showed the arm sagging). These are 25-35 kg*cm servos, same size.
SHOULDER_EFFORT = 3.0
ELBOW_EFFORT = 2.0
SERVO_VELOCITY = 1.5         # rad/s, deliberately well below the no-load 6 rad/s
LINEAR_DEFLECTION = 0.25     # mm, STL tessellation tolerance
ANGULAR_DEFLECTION = 0.6     # rad
WHITE, ANTHRACITE, BLUE = (0.96, 0.96, 0.97, 1), (0.18, 0.19, 0.20, 1), (0.08, 0.27, 0.50, 1)
COLOR = {  # white chassis and arm, anthracite joints and hardware, blue gripper fingers, yellow mecanum rollers
    'body': WHITE, 'deck': WHITE, 'arm': WHITE, 'accent': BLUE,
    'plate': ANTHRACITE, 'pcb': ANTHRACITE, 'motor': ANTHRACITE, 'metal': ANTHRACITE, 'servo': ANTHRACITE,
    'gear': ANTHRACITE, 'hub': ANTHRACITE, 'joint': ANTHRACITE, 'roller': (1.00, 0.78, 0.05, 1),
}
# ====================================================================================

doc = TDocStd_Document(TCollection_ExtendedString('doc'))
reader = STEPCAFControl_Reader(); reader.SetNameMode(True)
assert reader.ReadFile(STEP) == 1, 'cannot read ' + STEP
reader.Transfer(doc)
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def lname(label):
    n = TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else '?'

def loc_matrix(loc):
    t = loc.Transformation(); m = np.eye(4)
    for r in range(3):
        for c in range(4):
            m[r, c] = t.Value(r + 1, c + 1)
    return m

leaves = []
def walk(label, loc, path, wheel_origin):
    target = label
    if tool.IsReference_s(label):
        ref = TDF_Label(); tool.GetReferredShape_s(label, ref)
        loc = loc.Multiplied(tool.GetLocation_s(label)); target = ref
    name = lname(target)
    if name == 'Mecanum_wheel':
        wheel_origin = loc_matrix(loc)[:3, 3].copy()
    if tool.IsAssembly_s(target):
        seq = TDF_LabelSequence(); tool.GetComponents_s(target, seq)
        for i in range(1, seq.Length() + 1):
            walk(seq.Value(i), loc, path + [name], wheel_origin)
    else:
        leaves.append(dict(name=name, path=path + [name], shape=tool.GetShape_s(target).Moved(loc),
                           wheel_origin=wheel_origin))
roots = TDF_LabelSequence(); tool.GetFreeShapes(roots)
walk(roots.Value(1), TopLoc_Location(), [], None)

def props(shape):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g)
    c = g.CentreOfMass(); m = g.MatrixOfInertia()
    I = np.array([[m.Value(r, s) for s in (1, 2, 3)] for r in (1, 2, 3)])
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()]), I

def cylinders(shape):
    out = []; ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        a = BRepAdaptor_Surface(TopoDS.Face_s(ex.Current()))
        if a.GetType() == GeomAbs_Cylinder:
            cy = a.Cylinder(); d = cy.Axis().Direction(); o = cy.Axis().Location()
            out.append((cy.Radius(), np.array([d.X(), d.Y(), d.Z()]), np.array([o.X(), o.Y(), o.Z()])))
        ex.Next()
    return out

def tessellate(shape):
    BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION, False, ANGULAR_DEFLECTION, True)
    tris = []; ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current()); loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            m = loc_matrix(loc)
            pts = np.array([[tri.Node(i).X(), tri.Node(i).Y(), tri.Node(i).Z()]
                            for i in range(1, tri.NbNodes() + 1)])
            pts = pts @ m[:3, :3].T + m[:3, 3]
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                tris.append(pts[[a - 1, c - 1, b - 1] if flip else [a - 1, b - 1, c - 1]])
        ex.Next()
    return np.array(tris)

def write_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'] = n; rec['v'] = tris
    with open(path, 'wb') as f:
        f.write(b'amr cad_to_urdf'.ljust(80, b' ')); f.write(np.uint32(len(tris)).tobytes()); rec.tofile(f)

# ---------------------------- frames -----------------------------------------------
# CAD: X along the chassis, Y up, Z lateral.  ROS: x forward, y left, z up.
R_RC = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
by_name = {}
for l in leaves:
    by_name.setdefault(l['name'], []).append(l)
for l in leaves:
    l['vol'], l['com'], l['I'] = props(l['shape'])

servos = {}
for l in by_name['Servo']:
    c = l['com']
    key = ('waist' if c[0] < 5 and c[1] < 150 else 'shoulder' if c[0] < 60 else
           'elbow' if c[0] < 200 else 'wrist' if c[0] < 330 else 'gripper')
    servos[key] = l

def shaft_axis(leaf):
    """Servo output shaft: the r=10 mm boss cylinder of the servo model."""
    r, d, p = next(c for c in cylinders(leaf['shape']) if abs(c[0] - 10.0) < 0.05)
    return d / np.linalg.norm(d), p

u, shoulder_p = shaft_axis(servos['shoulder'])
if u[2] < 0:
    u = -u
ARM_YAW = math.atan2(u[0], u[2])      # how far the CAD pose has the arm turned about the waist
c_, s_ = math.cos(-ARM_YAW), math.sin(-ARM_YAW)
R_UNYAW = np.array([[c_, 0, s_], [0, 1, 0], [-s_, 0, c_]])
assert np.allclose(R_UNYAW @ u, [0, 0, 1], atol=1e-6), R_UNYAW @ u


rollers = [l for l in leaves if l['name'] == 'Roller.step']
wheel_r = 0.0
for l in rollers[:9]:
    t = tessellate(l['shape']).reshape(-1, 3); o = l['wheel_origin']
    wheel_r = max(wheel_r, np.hypot(t[:, 0] - o[0], t[:, 1] - o[1]).max())
AXLE_Y = 6.0
GROUND_Z = wheel_r - AXLE_Y           # z_ros = y_cad + GROUND_Z

def to_ros(p, moving):
    p = np.asarray(p, float)
    return ((R_RC @ (R_UNYAW if moving else np.eye(3))) @ p.T).T + np.array([0, 0, GROUND_Z])
def dir_ros(d, moving=True):
    v = R_RC @ (R_UNYAW if moving else np.eye(3)) @ np.asarray(d, float)
    return v / np.linalg.norm(v)

# ---------------------------- link assignment ----------------------------------------
LINK_OF_PART = {'base_1': ('base_link', 'body'), 'c2': ('base_link', 'deck'), 'c1': ('base_link', 'plate'),
                'pbc1': ('base_link', 'pcb'), 'orange_johnson_motor': ('base_link', 'motor'),
                'mount': ('base_link', 'metal'), 'bush_m_w2': ('base_link', 'metal'),
                'base': ('base_link', 'joint'), 'basecap': ('arm_turret', 'arm'), 'link1.1': ('arm_upper', 'arm'),
                'link2.1': ('arm_forearm', 'arm'), 'link3.1': ('arm_forearm', 'arm'), '4.1': ('arm_wrist', 'arm')}
SERVO_LINK = {'waist': 'base_link', 'shoulder': 'arm_turret', 'elbow': 'arm_forearm',
              'wrist': 'arm_forearm', 'gripper': 'arm_wrist'}
for k, l in servos.items():
    l['link'], l['mat'] = SERVO_LINK[k], 'servo'
parts_by_side = lambda name: sorted(by_name[name], key=lambda l: l['com'][2])
# CAD z- is ROS +y (left)
for name, base, mat in (('grip', 'gripper_finger', 'accent'), ('link5_2', 'gripper_rocker', 'gear'),
                        ('link5', 'gripper_gear', 'gear')):
    a, b = parts_by_side(name)
    a['link'], b['link'] = base + '_left', base + '_right'; a['mat'] = b['mat'] = mat
by_name['g2.1'][0]['link'] = 'gripper_gear_left'; by_name['g2.1'][0]['mat'] = 'gear'
by_name['g1'][0]['link'] = 'gripper_gear_right'; by_name['g1'][0]['mat'] = 'gear'
for l in leaves:
    if l['name'] in LINK_OF_PART:
        l['link'], l['mat'] = LINK_OF_PART[l['name']]

# ---------------------------- joint axes from shared bores -------------------------
def only(name, side=None):
    return by_name[name][0] if side is None else parts_by_side(name)[side]
def canon(p, d):
    d = d / np.linalg.norm(d); return p - d * p.dot(d)
def shared_pin(a, b, radius, tol=0.15):
    """Axis (point, direction) that parts a and b both have a cylinder of `radius` on."""
    for ra, da, pa in cylinders(a['shape']):
        if abs(ra - radius) > 0.05: continue
        for rb, db, pb in cylinders(b['shape']):
            if abs(rb - radius) > 0.05: continue
            if abs(abs(da.dot(db)) - 1) < 1e-3 and np.linalg.norm(canon(pa, da) - canon(pb, da)) < tol:
                return pa, da / np.linalg.norm(da)
    raise RuntimeError(f'no shared r={radius} between {a["path"]} and {b["path"]}')

def near(p, d, target):
    """Point on axis (p,d) closest to target (keeps the joint frame on the part)."""
    return p + d * (target - p).dot(d)

J = {}
J['waist'] = (np.array([0, 145.0, 0]), np.array([0, 1.0, 0]))
d, p = shaft_axis(servos['shoulder']); J['shoulder'] = (near(p, u, np.zeros(3)), u)
d, p = shaft_axis(servos['elbow']);    J['elbow'] = (near(p, u, np.zeros(3)), u)
d, p = shaft_axis(servos['wrist']);    J['wrist'] = (near(p, u, np.zeros(3)), u)
g_r, g_l = only('g1'), only('g2.1')
fi_l, fi_r = only('grip', 0), only('grip', 1)
ro_l, ro_r = only('link5_2', 0), only('link5_2', 1)
ln_l, ln_r = only('link5', 0), only('link5', 1)
d_g, p_g = shaft_axis(servos['gripper'])
if dir_ros(d_g)[0] < 0: d_g = -d_g
J['gear_right'] = (near(p_g, d_g, g_r['com']), d_g)
wrist41 = only('4.1')
pl = next(p for r, d, p in cylinders(g_l['shape']) if abs(r - 6.5) < .05)
J['gear_left'] = (near(pl, d_g, g_l['com']), d_g)
pD_r, _ = shared_pin(fi_r, ln_r, 1.5); J['finger_right'] = (near(pD_r, d_g, fi_r['com']), d_g)
pD_l, _ = shared_pin(fi_l, ln_l, 1.5); J['finger_left'] = (near(pD_l, d_g, fi_l['com']), d_g)
pB_r, _ = shared_pin(ro_r, wrist41, 1.5); J['rocker_right'] = (near(pB_r, d_g, ro_r['com']), d_g)
pB_l, _ = shared_pin(ro_l, wrist41, 1.5); J['rocker_left'] = (near(pB_l, d_g, ro_l['com']), d_g)

origin = {k: to_ros(v[0], True) for k, v in J.items()}
origin['waist'] = to_ros(J['waist'][0], False)
LINK_ORIGIN = {'base_link': np.zeros(3), 'arm_turret': origin['waist'], 'arm_upper': origin['shoulder'],
               'arm_forearm': origin['elbow'], 'arm_wrist': origin['wrist']}
# The CAD's gripper (its gears have unequal cranks, so the fingers do not meet on centre) is replaced by
# tools/make_gripper.py and urdf/gripper.xacro. Only its servo shaft position is still used.
OLD_GRIPPER = ('g1', 'g2.1', 'grip', 'link5', 'link5_2')
for l in leaves:
    if l['name'] in OLD_GRIPPER:
        l.pop('link', None)

# ---------------------------- tessellation + inertia per link -----------------------
os.makedirs(os.path.join(PKG, 'meshes', 'visual'), exist_ok=True)
os.makedirs(os.path.join(PKG, 'meshes', 'collision'), exist_ok=True)
groups = {}; inertial = {k: [] for k in LINK_ORIGIN}; allpts = {}
for l in leaves:
    if 'link' not in l: continue
    link = l['link']; moving = link != 'base_link'
    v = to_ros(tessellate(l['shape']).reshape(-1, 3), moving) - LINK_ORIGIN[link]
    tris = v.reshape(-1, 3, 3)
    groups.setdefault((link, l['mat']), []).append(tris)
    allpts.setdefault(link, []).append(tris)
    if l['name'] in FIXED_MASS:
        mass = FIXED_MASS[l['name']]
    else:
        mass = l['vol'] * PART_DENSITY.get(l['name'], PRINT_DENSITY)
    R = R_RC @ (R_UNYAW if moving else np.eye(3))
    com = (R @ l['com']) / 1000 + np.array([0, 0, GROUND_Z / 1000]) - LINK_ORIGIN[link] / 1000
    I = R @ (l['I'] / l['vol'] * mass * 1e-6) @ R.T
    inertial[link].append((mass, com, I))

def box_I(m, s):
    return np.diag([m / 12 * (s[1]**2 + s[2]**2), m / 12 * (s[0]**2 + s[2]**2), m / 12 * (s[0]**2 + s[1]**2)])
for name, m, xyz, size in EXTRA_BODIES:
    inertial['base_link'].append((m, np.array(xyz, float), box_I(m, size)))

def combine(items, min_mass=0.005):
    M = sum(i[0] for i in items); c = sum(i[0] * i[1] for i in items) / M
    I = np.zeros((3, 3))
    for m, p, Ii in items:
        d = p - c; I += Ii + m * (d.dot(d) * np.eye(3) - np.outer(d, d))
    if M < min_mass:
        I *= min_mass / M; M = min_mass
    # keep the small gripper bodies numerically friendly
    w = np.linalg.eigvalsh(I)
    if w[0] < 2e-7: I = I + np.eye(3) * (2e-7 - w[0])
    return M, c, I

INERTIAL = {k: combine(v) for k, v in inertial.items()}

# The round joint discs are part of the arm's CAD solids: triangles fully inside the disc radius around
# a joint axis (the link's y axis) get the joint colour.
JOINT_DISC_R = 28.5
PIVOTS = {'arm_turret': ['arm_upper'], 'arm_upper': ['arm_upper', 'arm_forearm'],
          'arm_forearm': ['arm_forearm', 'arm_wrist'], 'arm_wrist': ['arm_wrist']}
for link, pivots in PIVOTS.items():
    t = np.concatenate(groups[(link, 'arm')]); disc = np.zeros(len(t), bool)
    for p in pivots:
        c = LINK_ORIGIN[p] - LINK_ORIGIN[link]
        disc |= np.hypot(t[:, :, 0] - c[0], t[:, :, 2] - c[2]).max(1) < JOINT_DISC_R
    groups[(link, 'arm')] = [t[~disc]]; groups[(link, 'joint')] = [t[disc]]
mesh_files = {}
for (link, mat), parts in groups.items():
    t = np.concatenate(parts) / 1000.0
    fn = f'{link}__{mat}.stl'; write_stl(os.path.join(PKG, 'meshes', 'visual', fn), t)
    mesh_files.setdefault(link, []).append((fn, mat, len(t)))

# wheels: FR and RL match the CAD part. FL and RR are mirrored so the robot has the
# X roller pattern real mecanum kinematics needs (the CAD has 4 identical left wheels).
WHEELS = {'front_left': (1, -1, True), 'front_right': (1, 1, False),
          'rear_left': (-1, -1, False), 'rear_right': (-1, 1, True)}
wheel_info = {}
for wname, (sx, sz, mirror) in WHEELS.items():
    members = [l for l in leaves if l['wheel_origin'] is not None and np.sign(l['wheel_origin'][0]) == sx
               and np.sign(l['com'][2]) == sz]
    assert len(members) == 11, (wname, len(members))
    o = members[0]['wheel_origin'].copy()
    allv = np.concatenate([tessellate(l['shape']).reshape(-1, 3) for l in members])
    o[2] = (allv[:, 2].min() + allv[:, 2].max()) / 2
    for kind, mat, sel in (('hub', 'hub', lambda n: n != 'Roller.step'), ('roller', 'roller', lambda n: n == 'Roller.step')):
        t = np.concatenate([tessellate(l['shape']) for l in members if sel(l['name'])])
        v = (R_RC @ (t.reshape(-1, 3) - o).T).T / 1000
        if mirror:
            v[:, 0] *= -1
        t2 = v.reshape(-1, 3, 3)
        if mirror:
            t2 = t2[:, [0, 2, 1]]
        write_stl(os.path.join(PKG, 'meshes', 'visual', f'wheel_{wname}__{kind}.stl'), t2)
    wheel_info[wname] = dict(xyz=[sx * 82.5 / 1000, (-sz * abs(o[2])) / 1000, wheel_r / 1000], mirror=mirror)

# collision hulls (vertex-clustered convex hulls) for the moving arm links
def hull(tris, grid=4.0, max_points=120):
    pts = np.unique(np.round(tris.reshape(-1, 3) / grid) * grid, axis=0)
    h = ConvexHull(pts)
    return pts[h.simplices] / 1000.0, h
for link, parts in allpts.items():
    if link == 'base_link': continue
    tris = np.concatenate(parts)
    t, h = hull(tris, grid=2.0 if 'gripper' in link else 5.0)
    # outward winding
    c = tris.reshape(-1, 3).mean(0) / 1000.0
    nrm = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    flip = ((t.mean(1) - c) * nrm).sum(1) < 0
    t[flip] = t[flip][:, [0, 2, 1]]
    write_stl(os.path.join(PKG, 'meshes', 'collision', f'{link}.stl'), t)

np.savez_compressed(os.path.join(ROOT, 'tools', 'link_points.npz'),
                    **{k: np.concatenate(v).reshape(-1, 3) / 1000.0 for k, v in allpts.items()})

# ---------------------------- URDF emit ---------------------------------------------
def v3(a): return ' '.join(f'{x:.6f}' for x in a)
def inertial_xml(link):
    m, c, I = INERTIAL[link]
    return (f'    <inertial><origin xyz="{v3(c)}" rpy="0 0 0"/><mass value="{m:.5f}"/>'
            f'<inertia ixx="{I[0,0]:.8f}" ixy="{I[0,1]:.8f}" ixz="{I[0,2]:.8f}" iyy="{I[1,1]:.8f}" iyz="{I[1,2]:.8f}" izz="{I[2,2]:.8f}"/></inertial>\n')
def visuals_xml(link):
    s = ''
    for i, (fn, mat, n) in enumerate(mesh_files.get(link, [])):
        col = COLOR[mat]
        s += (f'    <visual name="{link}_{mat}"><geometry><mesh filename="package://amr_description/meshes/visual/{fn}"/></geometry>'
              f'<material name="amr_{mat}"><color rgba="{v3(col)}"/></material></visual>\n')
    return s

links_xml = ''
def add_link(name, collision=None):
    global links_xml
    coll = collision if collision is not None else (
        f'    <collision name="{name}_collision"><geometry><mesh filename="package://amr_description/meshes/collision/{name}.stl"/></geometry></collision>\n')
    links_xml += f'  <link name="{name}">\n{inertial_xml(name)}{visuals_xml(name)}{coll}  </link>\n\n'

# chassis collision: the box and the deck from the CAD extents, plus the arm's pedestal
bz0, bz1 = -8 + GROUND_Z, 90 + GROUND_Z
pz0, pz1 = 90 + GROUND_Z, 145 + GROUND_Z
base_coll = (f'    <collision name="chassis_collision"><origin xyz="0 0 {(bz0+bz1)/2000:.5f}"/><geometry><box size="0.295 0.215 {(bz1-bz0)/1000:.5f}"/></geometry></collision>\n'
             f'    <collision name="arm_pedestal_collision"><origin xyz="0 0 {(pz0+pz1)/2000:.5f}"/><geometry><cylinder radius="0.0605" length="{(pz1-pz0)/1000:.5f}"/></geometry></collision>\n')
add_link('base_link', base_coll)
for k in LINK_ORIGIN:
    if k == 'base_link': continue
    add_link(k)

def rel(child, parent):
    return (LINK_ORIGIN[child] - LINK_ORIGIN[parent]) / 1000.0

grip_axis = dir_ros(d_g)
AX = {'y': np.array([0, 1.0, 0]), 'z': np.array([0, 0, 1.0])}
def joint(name, parent, child, axis, lim, mimic=None, jtype='revolute', effort=SERVO_EFFORT, vel=SERVO_VELOCITY):
    m = f'<mimic joint="{mimic[0]}" multiplier="{mimic[1]:g}" offset="0"/>' if mimic else ''
    limit = f'lower="{lim[0]:.4f}" upper="{lim[1]:.4f}"'
    return (f'  <joint name="{name}" type="{jtype}"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{v3(rel(child, parent))}" rpy="0 0 0"/><axis xyz="{v3(axis)}"/>'
            f'<limit {limit} effort="{effort}" velocity="{vel}"/>'
            f'<dynamics damping="0.02" friction="0.01"/>{m}</joint>\n')

L = LIMITS
joints_xml = ''
joints_xml += joint('arm_waist_joint', 'base_link', 'arm_turret', AX['z'], L['waist'])
joints_xml += joint('arm_shoulder_joint', 'arm_turret', 'arm_upper', AX['y'], L['shoulder'], effort=SHOULDER_EFFORT)
joints_xml += joint('arm_elbow_joint', 'arm_upper', 'arm_forearm', AX['y'], L['elbow'], effort=ELBOW_EFFORT)
joints_xml += joint('arm_wrist_joint', 'arm_forearm', 'arm_wrist', AX['y'], L['wrist'])
GRIP_XYZ = (origin['gear_right'] - LINK_ORIGIN['arm_wrist']) / 1000
GRIP_PITCH = -math.atan2(grip_axis[2], grip_axis[0])

wheels_xml = ''
for w, info in wheel_info.items():
    wheels_xml += f'  <xacro:mecanum_wheel name="{w}" x="{info["xyz"][0]:.5f}" y="{info["xyz"][1]:.5f}" z="{info["xyz"][2]:.5f}" radius="{wheel_r/1000:.5f}" mass="{WHEEL_MASS}" width="0.025"/>\n'

xml = f'''<?xml version="1.0"?>
<!-- GENERATED by tools/cad_to_urdf.py from cad/mobile_base_1_robotic_arm.STEP. Do not edit; re-run the tool. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="wheel_radius" value="{wheel_r/1000:.5f}"/>
  <xacro:property name="grip_x" value="{GRIP_XYZ[0]:.6f}"/>
  <xacro:property name="grip_y" value="{GRIP_XYZ[1]:.6f}"/>
  <xacro:property name="grip_z" value="{GRIP_XYZ[2]:.6f}"/>
  <xacro:property name="grip_pitch" value="{GRIP_PITCH:.6f}"/>
  <xacro:property name="deck_z" value="{(90+GROUND_Z)/1000:.5f}"/>
  <xacro:macro name="amr_cad_body">
{links_xml}{joints_xml}
  </xacro:macro>
  <xacro:macro name="amr_cad_wheels">
{wheels_xml}  </xacro:macro>
</robot>
'''
os.makedirs(os.path.join(PKG, 'urdf', 'generated'), exist_ok=True)
open(os.path.join(PKG, 'urdf', 'generated', 'amr_cad.xacro'), 'w').write(xml)

# ---------------------------- exact CAD solids, for tools/verify_dimensions.py -------------
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
def exact_box(leafs):
    b = Bnd_Box()
    for l in leafs:
        BRepBndLib.AddOptimal_s(l['shape'], b, True, False)
    x0, y0, z0, x1, y1, z1 = b.Get()
    c = (R_RC @ np.array([[x0, y0, z0], [x1, y1, z1]]).T).T + np.array([0, 0, GROUND_Z])
    return [(c.min(0) / 1000).tolist(), (c.max(0) / 1000).tolist()]
cad_boxes = {link: exact_box([l for l in leaves if l.get('link') == link]) for link in LINK_ORIGIN}
for wname, (sx, sz, mirror) in WHEELS.items():
    cad_boxes['wheel_' + wname] = exact_box([l for l in leaves if l['wheel_origin'] is not None and np.sign(l['wheel_origin'][0]) == sx and np.sign(l['com'][2]) == sz])
for part in ('base_1', 'c1', 'c2', 'pbc1', 'base', 'basecap', 'link1.1', 'link2.1', 'link3.1', '4.1'):
    cad_boxes['part:' + part] = exact_box(by_name[part])
cad_boxes['whole_assembly'] = exact_box([l for l in leaves if l['name'] not in OLD_GRIPPER])

# ---------------------------- report ------------------------------------------------
tot = sum(v[0] for v in INERTIAL.values()) + 4 * WHEEL_MASS
report = dict(arm_cad_yaw_deg=math.degrees(ARM_YAW), wheel_radius_m=wheel_r / 1000, deck_z_m=(90 + GROUND_Z) / 1000,
              ground_clearance_m=(-8 + GROUND_Z) / 1000,
              wheels={k: v['xyz'] for k, v in wheel_info.items()},
              joints_m={k: (v / 1000).tolist() for k, v in origin.items()},
              gripper_axis=grip_axis.tolist(),
              masses_kg={k: round(v[0], 4) for k, v in INERTIAL.items()},
              coms_m={k: v[1].round(4).tolist() for k, v in INERTIAL.items()},
              link_origin_m={k: (v / 1000).tolist() for k, v in LINK_ORIGIN.items()},
              total_mass_kg=round(tot, 3), cad_boxes=cad_boxes,
              waist_servo_shaft_offset_mm=float(np.linalg.norm(canon(shaft_axis(servos['waist'])[1], np.array([0, 1.0, 0])))),
              meshes={f'{a}/{b}': sum(t.shape[0] for t in c) for (a, b), c in groups.items()})
json.dump(report, open(os.path.join(ROOT, 'tools', 'cad_report.json'), 'w'), indent=1)
print(f'wheel radius {wheel_r:.2f} mm, total mass {tot:.2f} kg')
print('triangles', sum(report['meshes'].values()))
