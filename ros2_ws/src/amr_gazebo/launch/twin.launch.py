"""Digital twin: Gazebo copies the real robot live, and RViz shows both, with real and simulated sensors.

Same pattern as the Spot Micro: robot computer and laptop on the same network and ROS_DOMAIN_ID.

  On the robot:   ros2 launch amr_control real.launch.py backend:=serial lidar:=true camera:=true
  On the laptop:  ros2 launch amr_gazebo twin.launch.py robot:=false

Everything on one computer, or to try it with no hardware:
  ros2 launch amr_gazebo twin.launch.py backend:=mock

Drive it as normal: ros2 run amr_control keyboard_teleop. Health: ros2 topic echo /twin/status
"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource as Src
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='true', description='also start the robot side here'),
        DeclareLaunchArgument('backend', default_value='mock'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mock_wheel_gain', default_value='1.0'),
        DeclareLaunchArgument('resync_distance', default_value='0.10'),
        IncludeLaunchDescription(Src(os.path.join(share('amr_control'), 'launch', 'real.launch.py')),
                                 condition=IfCondition(LC('robot')), launch_arguments={
            'backend': LC('backend'), 'port': LC('port'), 'joy': LC('joy'), 'lidar': LC('lidar'), 'camera': LC('camera'),
            'mock_wheel_gain': LC('mock_wheel_gain')}.items()),
        IncludeLaunchDescription(Src(os.path.join(share('amr_gazebo'), 'launch', 'sim.launch.py')), launch_arguments={
            'namespace': 'sim', 'gui': LC('gui'), 'rviz': 'false', 'kinematics': 'false', 'joy': 'false', 'nav': 'false'}.items()),
        Node(package='amr_control', executable='twin_sync', output='screen',
             parameters=[{'resync_distance': LC('resync_distance')}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LC('rviz')), output='log',
             arguments=['-d', os.path.join(share('amr_description'), 'rviz', 'twin.rviz')]),
    ])
