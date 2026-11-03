from glob import glob
from setuptools import setup
package = 'amr_control'
setup(
    name=package, version='1.0.0', packages=[package],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package]),
                ('share/' + package, ['package.xml']),
                ('share/' + package + '/config', glob('config/*.yaml')),
                ('share/' + package + '/launch', glob('launch/*.py'))],
    install_requires=['setuptools'], zip_safe=True, maintainer='hemanthros', license='Apache-2.0',
    entry_points={'console_scripts': [
        'mecanum_kinematics = amr_control.mecanum_kinematics:main',
        'keyboard_teleop = amr_control.keyboard_teleop:main',
        'joy_teleop = amr_control.joy_teleop:main',
        'hardware_bridge = amr_control.hardware_bridge:main',
        'twin_sync = amr_control.twin_sync:main',
        'mode_mux = amr_control.mode_mux:main',
        'scan_filter = amr_control.scan_filter:main',
        'sim_grasp_assist = amr_control.sim_grasp_assist:main',
        'object_detector = amr_control.object_detector:main',
        'mission_manager = amr_control.mission_manager:main',
        'mission = amr_control.mission_cli:main',
    ]})
