#!/usr/bin/env python3
"""Printable lidar and camera mounts (STEP for CAD, STL for printing and for the URDF).

All dimensions in mm. Frames: origin at the deck surface under the part, x forward, z up.
Lidar outline is the YDLIDAR X2 approximated (96 x 70 mm): check against your unit, the
plate uses slots so the screw pattern does not have to be exact.
"""
import os, site, sys
sys.path.append(site.getusersitepackages())
import cadquery as cq
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
OUT = os.path.join(ROOT, 'ros2_ws', 'src', 'amr_description', 'meshes', 'mounts')
CAD = os.path.join(ROOT, 'cad', 'mounts'); os.makedirs(OUT, exist_ok=True); os.makedirs(CAD, exist_ok=True)

STANDOFF = 15.0; PLATE = 3.0
# Lidar mount: slotted plate carried on four posts. Long side lateral, so it clears the arm pedestal.
plate = (cq.Workplane('XY').workplane(offset=STANDOFF).rect(66, 92).extrude(PLATE).edges('|Z').fillet(5)
         .faces('>Z').workplane().rect(48, 72, forConstruction=True).vertices().slot2D(9, 3.4, 90).cutThruAll()
         .faces('>Z').workplane().hole(22))        # cable pass-through
posts = (cq.Workplane('XY').rect(54, 80, forConstruction=True).vertices().circle(4).extrude(STANDOFF))
holes = (cq.Workplane('XY').rect(54, 80, forConstruction=True).vertices().circle(1.6).extrude(STANDOFF + PLATE))
lidar_mount = plate.union(posts).cut(holes)

# Camera bracket: foot on the deck, face tilted 15 deg down, fits a Pi Camera v2 / small USB module.
TILT = 15.0
foot = cq.Workplane('XY').box(14, 40, 3, centered=(True, True, False))
foot = foot.faces('>Z').workplane().pushPoints([(0, 14), (0, -14)]).hole(3.4)
face = (cq.Workplane('XY').box(3, 34, 34, centered=(True, True, False)).edges('|X and >Z').fillet(3)
        .faces('>X').workplane(centerOption='CenterOfBoundBox').rect(21, 12.5, forConstruction=True)
        .vertices().hole(2.2).faces('>X').workplane(centerOption='CenterOfBoundBox').rect(10, 14).cutThruAll())
face = face.rotate((0, 0, 0), (0, 1, 0), TILT).translate((-1.5, 0, 1.5))
gusset = (cq.Workplane('XZ').polyline([(-6.5, 3), (-1.5, 3), (-1.5, 12)]).close().extrude(1.5, both=True))
camera_mount = foot.union(face).union(gusset.translate((0, 10, 0))).union(gusset.translate((0, -10, 0)))

# Simple sensor bodies for visualisation only
lidar_body = (cq.Workplane('XY').rect(68, 94).extrude(16).edges('|Z').fillet(10)
              .union(cq.Workplane('XY').workplane(offset=16).center(0, 0).circle(33).extrude(22)))
camera_body = (cq.Workplane('XY').box(9, 25, 24).faces('>X').workplane().circle(4).extrude(4))

# ---- Branding on the front face, in the flat area between the slots, raised 0.5 mm. base_link frame.
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
LIFT = 0.5
FRONT = cq.Plane(origin=(147.5, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0))
def text_on(plane, txt, cap, cy):
    size = cap / 0.729   # DejaVu Sans Bold capital height is 0.729 em
    return (cq.Workplane(plane).center(0, cy)
            .text(txt, size, LIFT, fontPath=FONT, combine=False, halign='center', valign='center'))
brand_name = cq.Workplane(obj=cq.Compound.makeCompound(text_on(FRONT, 'MECHA', 16.5, 82.0).vals() + text_on(FRONT, 'MINDS', 16.5, 60.5).vals()))
brand_author = text_on(FRONT, 'SUBHANI', 7.5, 18.0)

for name, part in (('lidar_mount', lidar_mount), ('camera_mount', camera_mount)):
    cq.exporters.export(part, os.path.join(CAD, name + '.step'))
for name, part in (('lidar_mount', lidar_mount), ('camera_mount', camera_mount),
                   ('lidar_body', lidar_body), ('camera_body', camera_body),
                   ('brand_name', brand_name), ('brand_author', brand_author)):
    cq.exporters.export(part, os.path.join(OUT, name + '.stl'), tolerance=0.05, angularTolerance=0.3)
    bb = part.val().BoundingBox()
    print(name, f'x[{bb.xmin:.1f},{bb.xmax:.1f}] y[{bb.ymin:.1f},{bb.ymax:.1f}] z[{bb.zmin:.1f},{bb.zmax:.1f}] vol={part.val().Volume()/1000:.1f}cm3')
