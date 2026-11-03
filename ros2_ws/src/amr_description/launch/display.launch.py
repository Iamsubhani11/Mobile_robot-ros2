"""View the URDF in RViz with sliders for every joint: ros2 launch amr_description display.launch.py"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    urdf = xacro.process_file(os.path.join(share('amr_description'), 'urdf', 'amr.urdf.xacro')).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(urdf, value_type=str)}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share('amr_description'), 'rviz', 'amr.rviz')]),
    ])
