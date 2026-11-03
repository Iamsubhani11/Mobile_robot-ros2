"""Gazebo simulation of the robot.

  ros2 launch amr_gazebo sim.launch.py                 Gazebo + RViz
  ros2 launch amr_gazebo sim.launch.py joy:=true       + joystick control
  ros2 launch amr_gazebo sim.launch.py nav:=false      skip SLAM and Nav2
  ros2 launch amr_gazebo sim.launch.py gui:=false rviz:=false     headless
Keyboard needs its own terminal:  ros2 run amr_control keyboard_teleop
"""
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_sdf import GRASPABLE, urdf_to_sdf


def stop_leftover_servers():
    """A Gazebo server can outlive its launch (a closed terminal, a hard stop). A new launch on the same channel
    then hangs with a black Gazebo window and no controllers. Stops only this project's orphaned servers."""
    here = os.environ.get('AMENT_PREFIX_PATH', '')
    if 'MOBILE_ROBOT' not in here:
        return
    def env_of(pid):
        with open(f'/proc/{pid}/environ', 'rb') as f:
            return dict(e.split(b'=', 1) for e in f.read().split(b'\0') if b'=' in e)
    def cmd_of(pid):
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='ignore').strip()
    mine = (os.environ.get('ROS_DOMAIN_ID', ''), os.environ.get('GZ_PARTITION', ''))
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        try:
            if cmd_of(name) != 'gz sim server':
                continue
            with open(f'/proc/{name}/stat') as f:
                parent = f.read().rsplit(')', 1)[1].split()[1]
            if cmd_of(parent).startswith('gz sim'):
                continue          # still owned by a running launch
            env = env_of(name)
            theirs = (env.get(b'ROS_DOMAIN_ID', b'').decode(), env.get(b'GZ_PARTITION', b'').decode())
            if theirs == mine and b'MOBILE_ROBOT' in env.get(b'AMENT_PREFIX_PATH', b''):
                print(f'[sim.launch] stopping a leftover Gazebo server, pid {name}')
                os.kill(int(name), signal.SIGKILL)
        except (OSError, IndexError, ValueError):
            continue


def setup(context):
    ns = LC('namespace').perform(context).strip('/')
    gui = LC('gui').perform(context).lower() == 'true'
    stop_leftover_servers()
    world_name = 'arena'
    world = LC('world').perform(context) or os.path.join(share('amr_gazebo'), 'worlds', 'arena.sdf')
    controllers = os.path.join(share('amr_gazebo'), 'config', 'controllers.yaml')
    urdf = xacro.process_file(
        os.path.join(share('amr_description'), 'urdf', 'amr.urdf.xacro'),
        mappings={'sim': 'true', 'namespace': ns, 'controllers': controllers}).toxml()

    prefix = f'/{ns}/' if ns else '/'
    cm = f'{prefix}controller_manager'
    gz_args = ['gz', 'sim', '-r', world, '-v', '2'] if gui else ['gz', 'sim', '-r', '-s', '--headless-rendering', world, '-v', '2']
    gz = ExecuteProcess(cmd=gz_args, output='screen')

    rsp_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')] if ns else []
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
               namespace=ns, output='screen', remappings=rsp_remap,
               parameters=[{'robot_description': ParameterValue(urdf, value_type=str), 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', world_name, '-name', 'amr', '-string', urdf_to_sdf(urdf), '-x', LC('x'), '-y', LC('y'), '-z', '0.005'])

    # Everything Gazebo publishes uses /gz/...; here it gets its ROS name.
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', namespace=ns, output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/gz/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                             '/gz/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                             '/gz/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                             '/gz/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'] +
                            [f'/amr/{verb}/{c}@std_msgs/msg/Empty]gz.msgs.Empty' for verb in ('attach', 'detach') for c in GRASPABLE],
                  remappings=[('/gz/scan', f'{prefix}scan_raw'), ('/gz/camera/image_raw', f'{prefix}camera/image_raw'),
                              ('/gz/camera/camera_info', f'{prefix}camera/camera_info'),
                              ('/gz/ground_truth/odom', f'{prefix}ground_truth/odom')],
                  parameters=[{'use_sim_time': True}])

    def spawner(*names):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[*names, '--controller-manager', cm, '--controller-manager-timeout', '90', '--service-call-timeout', '60', '--switch-timeout', '60'],
                    parameters=[{'use_sim_time': True}])
    spawners = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
        # One spawner for all four: separate ones race on the controller manager while Nav2 starts up.
        TimerAction(period=1.0, actions=[spawner('joint_state_broadcaster', 'wheel_controller', 'arm_controller', 'gripper_controller')])]))

    assist = Node(package='amr_control', executable='sim_grasp_assist', namespace=ns, output='screen',
                  parameters=[{'use_sim_time': True}])
    scan_filter = Node(package='amr_control', executable='scan_filter', namespace=ns, parameters=[{'use_sim_time': True}])
    actions = [gz, rsp, spawn, bridge, spawners, assist, scan_filter]
    if LC('kinematics').perform(context).lower() == 'true':
        actions.append(Node(package='amr_control', executable='mecanum_kinematics', namespace=ns, output='screen',
                            parameters=[os.path.join(share('amr_control'), 'config', 'robot.yaml'), {'use_sim_time': True}]))
    if LC('kinematics').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('amr_control'), 'launch', 'core.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()))
    if LC('nav').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share('amr_control'), 'launch', 'nav.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()))
    if LC('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', output='log', parameters=[{'use_sim_time': True}],
                            arguments=['-d', os.path.join(share('amr_description'), 'rviz', 'amr.rviz')]))
    if LC('joy').perform(context).lower() == 'true':
        actions += [Node(package='joy', executable='joy_node', parameters=[{'use_sim_time': True, 'deadzone': 0.08, 'autorepeat_rate': 20.0}]),
                    Node(package='amr_control', executable='joy_teleop', output='screen',
                         parameters=[os.path.join(share('amr_control'), 'config', 'robot.yaml'), {'use_sim_time': True}])]
    return actions


def generate_launch_description():
    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share('amr_description'))),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('kinematics', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
