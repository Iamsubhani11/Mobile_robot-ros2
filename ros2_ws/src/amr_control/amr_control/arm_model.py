"""Arm limits, named poses, and a geometric guard that keeps teleop off the deck, lidar, camera and floor.

Numbers are read from the URDF so they never drift from the CAD.
"""
import math
import xml.etree.ElementTree as ET

ARM_JOINTS = ['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint']
GRIPPER = 'gripper_joint'
GRIPPER_RANGE = (0.0, 2.2)   # the commanded range; the simulated URDF's stops sit slightly past it
# q=0 is the CAD pose, the one in the project screenshot
POSES = {
    'home':  dict(arm_waist_joint=0.0, arm_shoulder_joint=0.0, arm_elbow_joint=0.0, arm_wrist_joint=0.0),
    # folded up and back: keeps the lidar plane and camera view clear while driving
    'stow':  dict(arm_waist_joint=0.0, arm_shoulder_joint=-0.95, arm_elbow_joint=-0.6, arm_wrist_joint=0.9),
    # gripper straight down, 10 cm above the floor at the grasp spot
    'reach': dict(arm_waist_joint=0.0, arm_shoulder_joint=0.045, arm_elbow_joint=0.089, arm_wrist_joint=0.354),
}
DECK_Z = 0.1084
FOOT_X, FOOT_Y = 0.1525, 0.1125     # chassis half extents plus 5 mm


def _rotate(axis, a, p):
    n = math.sqrt(sum(v * v for v in axis)); k = [v / n for v in axis]
    c, s = math.cos(a), math.sin(a); d = sum(k[i] * p[i] for i in range(3))
    x = [k[1] * p[2] - k[2] * p[1], k[2] * p[0] - k[0] * p[2], k[0] * p[1] - k[1] * p[0]]
    return [p[i] * c + x[i] * s + k[i] * d * (1 - c) for i in range(3)]


class ArmModel:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.limits = {}; self.org = {}
        for j in root.findall('joint'):
            xyz = j.find('origin').get('xyz', '0 0 0').split() if j.find('origin') is not None else [0, 0, 0]
            self.org[j.get('name')] = [float(v) for v in xyz]
            if j.get('name') == GRIPPER:
                self.grip_axis = [float(v) for v in j.find('axis').get('xyz').split()]
            lim = j.find('limit')
            if lim is not None and lim.get('lower') is not None:
                self.limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))

    def clamp(self, joint, value):
        lo, hi = GRIPPER_RANGE if joint == GRIPPER else self.limits[joint]
        return max(lo, min(hi, value))

    def key_points(self, q):
        """World points along the arm, in the base frame, densely enough to catch the links."""
        wa, sh, el, wr = (q[j] for j in ARM_JOINTS)
        waist = self.org['arm_waist_joint']
        o_sh, o_el, o_wr = self.org['arm_shoulder_joint'], self.org['arm_elbow_joint'], self.org['arm_wrist_joint']
        o_tcp = self.org['gripper_tcp_joint']
        y = (0, 1, 0)
        def in_upper(p): return [o_sh[i] + _rotate(y, sh, p)[i] for i in range(3)]
        def in_fore(p):
            r = _rotate(y, el, p)
            return in_upper([o_el[i] + r[i] for i in range(3)])
        def in_wrist(p):
            r = _rotate(y, wr, p)
            return in_fore([o_wr[i] + r[i] for i in range(3)])
        pts = []
        for t in (0.25, 0.5, 0.75, 1.0):
            pts.append(in_upper([o_el[i] * t for i in range(3)]))
            pts.append(in_fore([o_wr[i] * t for i in range(3)]))
        # fingers: tip plus a lateral spread at full opening
        for t in (0.4, 0.7, 1.0):
            for lat in (-0.07, 0.0, 0.07):
                p = [o_tcp[0] * t, lat, o_tcp[2] * t]
                pts.append(in_wrist(p))
        return [[waist[i] + _rotate((0, 0, 1), wa, p)[i] for i in range(3)] for p in pts]

    def collides(self, q):
        for x, y, z in self.key_points(q):
            if z < 0.004:
                return True
            if abs(x) < FOOT_X and abs(y) < FOOT_Y and z < DECK_Z + 0.015:
                return True
            if 0.055 < x < 0.150 and abs(y) < 0.060 and z < DECK_Z + 0.075:   # lidar and camera
                return True
        return False

    def solve_ik(self, x, y, z, pitch_deg=90.0):
        """Fingertip midpoint at (x, y, z) in the base frame, fingers pitch_deg below horizontal.

        Closed form: the waist takes the sideways offset, then a planar shoulder-elbow chain
        reaches the wrist. Returns {joint: angle} or None when out of reach or past a limit.
        """
        waist_o, sh_o = self.org['arm_waist_joint'], self.org['arm_shoulder_joint']
        L1, L2, T = self.org['arm_elbow_joint'], self.org['arm_wrist_joint'], self.org['gripper_tcp_joint']
        rho2 = x * x + y * y
        if rho2 <= T[1] * T[1]:
            return None
        r = math.sqrt(rho2 - T[1] * T[1])
        psi = math.atan2(y, x) - math.asin(T[1] / math.sqrt(rho2))
        # finger direction at q=0 lies in the gripper plane, perpendicular to the gear axis
        p0 = math.atan2(abs(self.grip_axis[0]), abs(self.grip_axis[2]))
        theta = math.radians(pitch_deg) - p0
        c, s = math.cos(theta), math.sin(theta)
        wx = r - (T[0] * c + T[2] * s)
        wz = z - (-T[0] * s + T[2] * c)
        vx, vz = wx - sh_o[0], wz - (waist_o[2] + sh_o[2])
        a, b = math.hypot(L1[0], L1[2]), math.hypot(L2[0], L2[2])
        d = math.hypot(vx, vz)
        if d > a + b or d < abs(a - b) or d == 0:
            return None
        elev1 = math.atan2(vz, vx) + math.acos((a * a + d * d - b * b) / (2 * a * d))     # elbow up
        elev2 = math.atan2(vz - a * math.sin(elev1), vx - a * math.cos(elev1))
        sh = math.atan2(L1[2], L1[0]) - elev1
        el = math.atan2(L2[2], L2[0]) - sh - elev2
        q = {'arm_waist_joint': math.atan2(math.sin(psi), math.cos(psi)), 'arm_shoulder_joint': sh,
             'arm_elbow_joint': el, 'arm_wrist_joint': theta - sh - el}
        for j, v in q.items():
            lo, hi = self.limits[j]
            if not lo - 1e-3 <= v <= hi + 1e-3:
                return None
        return q

    def tcp(self, q):
        """Fingertip midpoint position for joint angles q, base frame."""
        y = (0, 1, 0)
        o_sh, o_el, o_wr, o_t = (self.org[n] for n in ('arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint', 'gripper_tcp_joint'))
        p = _rotate(y, q['arm_wrist_joint'], o_t)
        p = _rotate(y, q['arm_elbow_joint'], [o_wr[i] + p[i] for i in range(3)])
        p = _rotate(y, q['arm_shoulder_joint'], [o_el[i] + p[i] for i in range(3)])
        p = _rotate((0, 0, 1), q['arm_waist_joint'], [o_sh[i] + p[i] for i in range(3)])
        w = self.org['arm_waist_joint']
        return [w[i] + p[i] for i in range(3)]
