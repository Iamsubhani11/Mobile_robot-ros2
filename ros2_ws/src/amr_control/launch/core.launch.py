"""Mode arbiter, object detector and mission manager. Runs identically in simulation and on the robot."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    t = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='amr_control', executable='mode_mux', output='screen', parameters=[t]),
        Node(package='amr_control', executable='object_detector', output='screen', parameters=[t]),
        Node(package='amr_control', executable='mission_manager', output='screen', parameters=[t]),
    ])
