"""URDF -> SDF for spawning, adding what URDF cannot express.

Mecanum rollers are modelled the way Gazebo's own mecanum demo does it: a sphere with friction
only along the direction the wheel can push (fdir1, fixed in the chassis frame). The roller
is free in the other direction. URDF has no syntax for this and <gazebo> collision blobs are
dropped by the converter, so it is added to the converted SDF.
"""
import re
import subprocess
import tempfile

GRASPABLE = ('red', 'green', 'blue')
ROLLER_FDIR = {'front_left': '1 -1 0', 'front_right': '1 1 0', 'rear_left': '1 1 0', 'rear_right': '1 -1 0'}


def urdf_to_sdf(urdf):
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(urdf); f.flush()
        sdf = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True, check=True).stdout
    for wheel, fdir in ROLLER_FDIR.items():
        pattern = rf"(<collision name='{wheel}_wheel[^']*'>.*?</geometry>)\s*(?:<surface>.*?</surface>\s*)?(</collision>)"
        surface = (f"<surface><friction><ode><mu>1.0</mu><mu2>0.0</mu2>"
                   f"<fdir1 xmlns:gz='http://gazebosim.org/schema' gz:expressed_in='base_link'>{fdir}</fdir1>"
                   f"</ode></friction></surface>")
        sdf, n = re.subn(pattern, lambda m: m.group(1) + surface + m.group(2), sdf, flags=re.S)
        if n != 1:
            raise RuntimeError(f'wheel collision for {wheel} not found in the SDF')
    # grippy finger pads, like rubber on printed plastic
    pad = "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface>"
    sdf, n = re.subn(r"(<collision name='gripper_finger_(?:left|right)[^']*'>.*?</geometry>)\s*(?:<surface>.*?</surface>\s*)?(</collision>)",
                     lambda m: m.group(1) + pad + m.group(2), sdf, flags=re.S)
    if n != 2:
        raise RuntimeError('finger collisions not found in the SDF')
    # Sim-only grasp: DART does not deliver enough normal force to hold an object by friction.
    # sim_grasp_assist joins a block to the finger when the fingers have really closed on it.
    blocks = ''.join(
        f"<plugin filename='gz-sim-detachable-joint-system' name='gz::sim::systems::DetachableJoint'>"
        f"<parent_link>gripper_finger_right</parent_link><child_model>block_{c}</child_model><child_link>link</child_link>"
        f"<attach_topic>/amr/attach/{c}</attach_topic><detach_topic>/amr/detach/{c}</detach_topic></plugin>"
        for c in GRASPABLE)
    sdf = sdf.replace('</model>', blocks + '</model>', 1) if sdf.count('</model>') == 1 else sdf
    # gz_ros2_control drives the gripper mimic joints itself; DART cannot use SDF mimic constraints.
    return re.sub(r'<mimic .*?</mimic>', '', sdf, flags=re.S)
