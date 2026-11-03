"""The real robot. Run this on the computer connected to the hardware (normally the Pi).

  ros2 launch amr_control real.launch.py backend:=serial port:=/dev/ttyUSB0       Uno, ESP32, UNO Q
  ros2 launch amr_control real.launch.py backend:=pi_gpio                         Pi driving everything itself
  ros2 launch amr_control real.launch.py backend:=mock                            no hardware

Options: lidar:=true lidar_port:=/dev/ttyUSB1   camera:=true camera_device:=/dev/video0
         nav:=true (SLAM + Nav2)  joy:=true  rviz:=true
"""
import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource as Src
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def have(pkg):
    try:
        share(pkg)
        return True
    except PackageNotFoundError:
        return False


def sensors(context):
    out = []
    if LC('lidar').perform(context).lower() == 'true':
        if have('ydlidar_ros2_driver'):
            out.append(Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
                            name='ydlidar_ros2_driver_node', output='screen',
                            parameters=[os.path.join(share('amr_control'), 'config', 'ydlidar_x2.yaml'),
                                        {'port': LC('lidar_port').perform(context)}],
                            remappings=[('scan', 'scan_raw')]))
        else:
            out.append(LogInfo(msg='lidar:=true but ydlidar_ros2_driver is not installed: see docs/WIRING.md'))
    if LC('camera').perform(context).lower() == 'true':
        if have('v4l2_camera'):
            out.append(Node(package='v4l2_camera', executable='v4l2_camera_node', name='camera', output='screen',
                            parameters=[{'video_device': LC('camera_device').perform(context), 'image_size': [640, 480],
                                         'camera_frame_id': 'camera_optical_frame', 'pixel_format': 'YUYV'}],
                            remappings=[('image_raw', 'camera/image_raw'), ('camera_info', 'camera/camera_info')]))
        else:
            out.append(LogInfo(msg='camera:=true but v4l2_camera is not installed: sudo apt install ros-humble-v4l2-camera'))
    return out


def generate_launch_description():
    urdf = xacro.process_file(os.path.join(share('amr_description'), 'urdf', 'amr.urdf.xacro')).toxml()
    robot = os.path.join(share('amr_control'), 'config', 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='serial'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mock_wheel_gain', default_value='1.0'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(urdf, value_type=str)}]),
        Node(package='amr_control', executable='hardware_bridge', output='screen', parameters=[robot, {
            'backend': LC('backend'), 'port': LC('port'),
            'baud': ParameterValue(LC('baud'), value_type=int),
            'mock_wheel_gain': ParameterValue(LC('mock_wheel_gain'), value_type=float)}]),
        Node(package='amr_control', executable='mecanum_kinematics', output='screen', parameters=[robot]),
        Node(package='amr_control', executable='scan_filter'),
        IncludeLaunchDescription(Src(os.path.join(share('amr_control'), 'launch', 'core.launch.py'))),
        IncludeLaunchDescription(Src(os.path.join(share('amr_control'), 'launch', 'nav.launch.py')), condition=IfCondition(LC('nav'))),
        OpaqueFunction(function=sensors),
        Node(package='joy', executable='joy_node', condition=IfCondition(LC('joy')),
             parameters=[{'deadzone': 0.08, 'autorepeat_rate': 20.0}]),
        Node(package='amr_control', executable='joy_teleop', output='screen', condition=IfCondition(LC('joy')), parameters=[robot]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LC('rviz')), output='log',
             arguments=['-d', os.path.join(share('amr_description'), 'rviz', 'amr.rviz')]),
    ])
