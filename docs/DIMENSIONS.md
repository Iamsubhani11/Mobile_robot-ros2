# Dimension check: URDF against your CAD

CAD column: exact solids in `cad/mobile_base_1_robotic_arm.STEP`. URDF column: the finished robot's meshes placed by its joints, with the waist turned to the CAD pose.
Regenerate: `python3 tools/verify_dimensions.py robot.urdf docs/DIMENSIONS.md`.

| Part | CAD size x × y × z (mm) | URDF size (mm) | Size difference (mm) | Position difference (mm) |
|---|---|---|---|---|
| Whole robot (without lidar, camera, branding and gripper) | 537.79 × 289.25 × 322.59 | 537.67 × 288.80 × 322.16 | 0.45 | 0.26 |
| Chassis + motors + arm pedestal (base_link) | 295.24 × 271.03 × 153.26 | 295.00 × 271.00 × 153.00 | 0.26 | 0.19 |
| Arm turret (basecap + shoulder servo) | 100.53 × 97.14 × 65.81 | 100.29 × 96.90 × 65.57 | 0.25 | 0.12 |
| Upper arm (link1.1) | 180.79 × 37.06 × 125.44 | 180.67 × 36.90 × 125.13 | 0.32 | 0.16 |
| Forearm (link2.1 + link3.1 + 2 servos) | 230.32 × 69.27 × 79.88 | 230.06 × 68.99 × 79.43 | 0.44 | 0.28 |
| Wrist (4.1 + gripper servo) | 104.21 × 58.16 × 79.87 | 104.13 × 58.09 × 79.73 | 0.14 | 0.09 |
| Wheel front_left | 48.70 × 25.34 × 48.92 | 48.36 × 25.00 × 48.58 | 0.34 | 0.33 |
| Wheel front_right | 48.70 × 25.34 × 48.92 | 48.36 × 25.00 × 48.58 | 0.34 | 0.58 |
| Wheel rear_left | 48.70 × 25.34 × 48.92 | 48.36 × 25.00 × 48.58 | 0.34 | 0.58 |
| Wheel rear_right | 48.70 × 25.34 × 48.92 | 48.36 × 25.00 × 48.58 | 0.34 | 0.33 |

Largest size difference: **0.45 mm**.

Key measurements read from the CAD:

- Chassis: 295 × 215 mm, deck 108.4 mm above the ground, 10.4 mm ground clearance
- Wheels: 48.8 mm diameter, axles at x = ±82.5 mm, wheel centres at y = ±131.9 mm
- Arm: waist at z = 163.4 mm, shoulder to elbow 134.0 mm, elbow to wrist 179.0 mm

The STL files are tessellated at 0.25 mm, so a few hundredths of a millimetre of difference is expected. FL and RR wheels are mirrored and each wheel is centred on its bushing axis (the CAD has two of them 0.2 to 0.4 mm off): see `docs/CAD_FINDINGS.md`.
