#!/usr/bin/env python3
"""Offscreen CAD-style renderer: reads the real URDF, applies joint values, draws to PNG.
Used for inspection images and for checking joint directions without opening a GUI.
Library use: load_robot(urdf_path), fk(robot, q), render(robot, q, eye, target, out)"""
import os, sys, xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

def rpy_m(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
def origin_T(e):
    T = np.eye(4)
    if e is not None:
        T[:3, 3] = [float(x) for x in e.get('xyz', '0 0 0').split()]
        T[:3, :3] = rpy_m(*[float(x) for x in e.get('rpy', '0 0 0').split()])
    return T
def axis_T(axis, a):
    k = axis / np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4); T[:3, :3] = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    return T
def read_stl(path, scale=1.0):
    with open(path, 'rb') as f:
        head = f.read(80); n = np.frombuffer(f.read(4), '<u4')[0]
        rec = np.fromfile(f, dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')], count=n)
    if rec.size != n or head[:5] == b'solid' and rec.size == 0:
        raise ValueError(path)
    return rec['v'].astype(float) * scale

def read_any_stl(path, scale=1.0):
    head = open(path, 'rb').read(5)
    if head == b'solid':
        txt = open(path, errors='ignore').read()
        if 'facet' in txt[:400]:
            v = [[float(x) for x in l.split()[1:4]] for l in txt.splitlines() if l.strip().startswith('vertex')]
            return np.array(v).reshape(-1, 3, 3) * scale
    return read_stl(path, scale)

def load_robot(urdf):
    root = ET.parse(urdf).getroot(); pkg_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2_ws', 'src')
    links, joints = {}, {}
    for l in root.findall('link'):
        vis = []
        for v in l.findall('visual'):
            m = v.find('geometry/mesh')
            if m is None: continue
            path = m.get('filename').replace('package://', pkg_root + '/')
            s = m.get('scale'); scale = float(s.split()[0]) if s else 1.0
            col = v.find('material/color')
            rgba = [float(x) for x in col.get('rgba').split()] if col is not None else [.7, .7, .7, 1]
            vis.append((read_any_stl(path, scale), origin_T(v.find('origin')), np.array(rgba[:3])))
        links[l.get('name')] = vis
    for j in root.findall('joint'):
        ax = j.find('axis'); mim = j.find('mimic'); lim = j.find('limit')
        joints[j.find('child').get('link')] = dict(
            name=j.get('name'), parent=j.find('parent').get('link'), T=origin_T(j.find('origin')), type=j.get('type'),
            axis=np.array([float(x) for x in ax.get('xyz').split()]) if ax is not None else None,
            mimic=(mim.get('joint'), float(mim.get('multiplier', 1))) if mim is not None else None,
            lim=(float(lim.get('lower', 0)), float(lim.get('upper', 0))) if lim is not None else None)
    return dict(links=links, joints=joints)

def fk(robot, q, base=np.eye(4)):
    done = {'base_link': base}; J = robot['joints']
    def pose(link):
        if link in done: return done[link]
        j = J[link]; a = 0.0
        if j['type'] != 'fixed':
            a = q.get(j['mimic'][0], 0.0) * j['mimic'][1] if j['mimic'] else q.get(j['name'], 0.0)
        T = pose(j['parent']) @ j['T']
        if j['axis'] is not None:
            if j['type'] == 'prismatic':
                S = np.eye(4); S[:3, 3] = np.asarray(j['axis'], float) * a; T = T @ S
            else:
                T = T @ axis_T(j['axis'], a)
        done[link] = T; return T
    for l in robot['links']: pose(l)
    return done

def look_at(eye, target, up=(0, 0, 1)):
    f = np.array(target, float) - eye; f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r); u = np.cross(r, f)
    return np.array([r, u, -f]), np.array(eye, float)

def render(robot, q, eye, target, out=None, size=(1400, 1000), fov=30, ortho=None, bg=(226, 229, 234), grid=True, title=None, ground=True):
    W, H = size; R, e = look_at(eye, target, up=(0, 0, 1) if abs(np.subtract(target, eye)[2]) < 0.99 * np.linalg.norm(np.subtract(target, eye)) else (1, 0, 0))
    Z = np.full((H, W), np.inf); C = np.zeros((H, W, 3)); C[:] = np.array(bg) / 255.; N = np.zeros((H, W, 3))
    f = (H / 2) / np.tan(np.radians(fov) / 2)
    def proj(p):
        c = (p - e) @ R.T; d = -c[..., 2]
        if ortho:
            s = H / ortho; return W/2 + c[..., 0] * s, H/2 - c[..., 1] * s, d
        return W/2 + f * c[..., 0] / np.maximum(d, 1e-6), H/2 - f * c[..., 1] / np.maximum(d, 1e-6), d
    light = np.array([0.35, -0.45, 0.82]); light /= np.linalg.norm(light)
    poses = fk(robot, q)
    for link, vis in robot['links'].items():
        for tris, To, rgb in vis:
            T = poses[link] @ To
            P = tris @ T[:3, :3].T + T[:3, 3]
            n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
            x, y, d = proj(P.reshape(-1, 3)); x = x.reshape(-1, 3); y = y.reshape(-1, 3); d = d.reshape(-1, 3)
            for i in range(len(P)):
                x0, x1 = int(max(np.floor(x[i].min()), 0)), int(min(np.ceil(x[i].max()) + 1, W))
                y0, y1 = int(max(np.floor(y[i].min()), 0)), int(min(np.ceil(y[i].max()) + 1, H))
                if x0 >= x1 or y0 >= y1: continue
                xs, ys = np.meshgrid(np.arange(x0, x1) + .5, np.arange(y0, y1) + .5)
                a = (x[i,1]-x[i,0])*(y[i,2]-y[i,0]) - (x[i,2]-x[i,0])*(y[i,1]-y[i,0])
                if abs(a) < 1e-9: continue
                w0 = ((x[i,1]-xs)*(y[i,2]-ys) - (x[i,2]-xs)*(y[i,1]-ys)) / a
                w1 = ((x[i,2]-xs)*(y[i,0]-ys) - (x[i,0]-xs)*(y[i,2]-ys)) / a
                w2 = 1 - w0 - w1
                m = (w0 >= -1e-3) & (w1 >= -1e-3) & (w2 >= -1e-3)
                if not m.any(): continue
                dz = w0*d[i,0] + w1*d[i,1] + w2*d[i,2]
                sub = Z[y0:y1, x0:x1]; m &= dz < sub
                if not m.any(): continue
                sub[m] = dz[m]
                nn = n[i] if n[i].dot(e - P[i, 0]) > 0 else -n[i]
                s = 0.35 + 0.65 * max(nn.dot(light), 0) * 1.0 + 0.15 * max(nn.dot((e - P[i,0]) / np.linalg.norm(e - P[i,0])), 0)
                C[y0:y1, x0:x1][m] = np.clip(rgb * s, 0, 1); N[y0:y1, x0:x1][m] = nn
    if grid:
        ys, xs = np.mgrid[0:H, 0:W]; hit = np.isinf(Z)
        rx = (xs + .5 - W/2) / f; ry = -(ys + .5 - H/2) / f
        dirs = rx[..., None] * R[0] + ry[..., None] * R[1] - R[2]
        t = -e[2] / np.where(np.abs(dirs[..., 2]) < 1e-9, 1e-9, dirs[..., 2])
        gx, gy = e[0] + t * dirs[..., 0], e[1] + t * dirs[..., 1]
        for step, wid, col in ((0.1, .0012, .25), (0.5, .002, .5)):
            on = (np.abs((gx + step/2) % step - step/2) < wid * np.maximum(t, 1e-3) * 1.2) | (np.abs((gy + step/2) % step - step/2) < wid * np.maximum(t, 1e-3) * 1.2)
            C[hit & (t > 0) & on] = C[hit & (t > 0) & on] * (1 - col) + np.array([.45, .48, .54]) * col
    # CAD silhouette lines from normal and depth discontinuities
    dn = np.zeros((H, W)); dz = np.zeros((H, W))
    dn[:-1, :-1] = np.maximum(1 - (N[:-1, :-1] * N[1:, :-1]).sum(-1), 1 - (N[:-1, :-1] * N[:-1, 1:]).sum(-1))
    Zf = np.where(np.isinf(Z), 100, Z)
    dz[:-1, :-1] = np.maximum(np.abs(Zf[:-1, :-1] - Zf[1:, :-1]), np.abs(Zf[:-1, :-1] - Zf[:-1, 1:]))
    C[(dn > 0.25) | (dz > 0.012)] *= 0.45
    img = Image.fromarray((C * 255).astype(np.uint8))
    if out: img.save(out)
    return img
