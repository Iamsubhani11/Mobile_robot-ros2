# Gripper: print and build guide

A parallel-jaw rack-and-pinion gripper that replaces parts `g1`, `g2.1`, `link5`, `link5_2` and `grip`. Your arm, wrist part `4.1` and gripper servo stay as they are.

The servo turns one central gear, which drives two racks in opposite directions. Both jaws always move the same distance, stay parallel and meet on the centre line.

| Spec | Value |
|---|---|
| Pad gap | 56 mm open to 1 mm closed |
| Gap formula | `gap (mm) = 56 - 25 × q`, with q the servo angle in rad |
| Servo travel | 2.2 rad (126°), pulses 800 to 2200 µs |
| Grip force | About 40 N per jaw at 1 N·m servo stall torque (calculated) |
| Gear | Module 1.25, 20 teeth, 20° pressure angle, 25 mm pitch diameter |

**Status:** checked in CAD and Gazebo, but not built yet. Print the gear and one rack first and check the mesh before printing the rest.

## Parts in `gripper_stl/`

Already laid flat for printing, units mm. Print two racks; one of each other part.

| File | Qty | Material | Settings |
|---|---|---|---|
| `pinion.stl` | 1 | PETG or PLA | 0.12 mm layers, 100% infill |
| `rack.stl` | **2** | PETG or PLA | 0.12 mm layers, 100% infill |
| `base.stl` | 1 | PETG or PLA | 0.2 mm layers, 4 walls, 40% infill |
| `cover.stl` | 1 | PETG or PLA | 0.2 mm layers, 4 walls, 40% infill |
| `finger_left.stl` | 1 | PETG or PLA | 0.2 mm layers, 5 walls, 50% infill |
| `finger_right.stl` | 1 | PETG or PLA | 0.2 mm layers, 5 walls, 50% infill |
| `pad_left_tpu.stl` | 1 | TPU 95A | Grid face up |
| `pad_right_tpu.stl` | 1 | TPU 95A | Grid face up |

## Hardware

| Item | Qty | Use |
|---|---|---|
| M3 × 16 screw and nut | 2 | Base to wrist part `4.1` |
| M3 × 16 screw | 4 | Fingers to racks |
| M3 × 10 screw | 4 | Cover to base |
| Round servo horn, up to Ø20 mm | 1 | Pinion to servo |
| M2 × 6 screw | 4 | Pinion to horn |
| Grease (PTFE or silicone) | A little | Racks and gear |
| Glue (cyanoacrylate) | A little | Pads to fingers |

The pinion has M2 holes on a 14 mm circle. Measure your horn first. If its holes differ, drill new ones in the pinion's hub.

## Assembly

1. Remove the old gears, links and fingers from the wrist. Keep the servo in place.
2. Fit the base over the raised boss on `4.1`. Fix it with the two M3 screws through the two existing 3 mm holes, with nuts on the servo side. The holes are 3 mm, so ream them slightly if an M3 screw won't pass.
3. Power the servo and send **1500 µs**, which is the middle of the gripper's travel (q = 1.1 rad).
4. Screw the horn to the pinion and press it onto the servo spline without turning the shaft. Tighten the horn screw.
5. Slide both racks into their channels from the ends. Set them so the **fingers will be 28.5 mm apart**, the correct gap at the servo's midpoint, and so they mesh with the gear. If the gap comes out wrong, move each rack one tooth at a time (3.9 mm).
6. Screw on the cover. Check the racks slide freely, and sand the channels lightly if they bind.
7. Bolt the fingers to the racks through the cover slots. `finger_left` (the shorter one) goes on the lower rack, `finger_right` on the upper rack.
8. Glue the pads to the inner faces of the fingertips.

## Check it

1. Run `ros2 run amr_control keyboard_teleop`, then tap `b` to close and `g` to open.
2. If `b` opens the jaws, set `SERVO_SIGN[4] = -1` and `SERVO_CENTER_US[4] = 2200` in `firmware/amr_firmware/config.h`. For the Pi, make the same change in `backends.py`.
3. Measure the gap at both ends of travel against the formula above.
4. Set `grip_closed` to suit your object, using `q = (56 - object width) / 25` plus about 0.05 for squeeze. Holding far past that stalls the servo and heats it.

## Why this design

| Requirement | Old gripper | New gripper |
|---|---|---|
| Fingers meet on centre | No, unequal gear cranks | Yes, one gear drives both |
| Jaw faces parallel | Only roughly | Yes, at every width |
| Same parts in your other zips? | Yes, identical files | Replaced |
| Grip surface | Hard plastic | TPU pad with shallow grid |

Design checks run on these exact parts:
- **Gear mesh and sliding parts:** no overlap at five positions across full travel.
- **Wrist and servo:** nothing passes through them, open or closed.
- **Forearm:** clear over the wrist range, so the wrist's upper limit was lowered from 1.15 to 1.10 rad.
- **Lidar:** fingers were shortened by 25 mm so they stay above the scan plane at home. The first version put them in the scan and blocked navigation.

The real printer clearance can't be checked in software, so that's the first thing to test. To change a dimension, edit `tools/make_gripper.py` and run it to regenerate the STLs.
