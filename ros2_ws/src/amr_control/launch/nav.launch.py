"""SLAM + Nav2 for the holonomic base. Builds the map as it goes, so no saved map is needed.

The controller is DWB with sideways sampling so Nav2 actually uses strafing. Its output goes
to cmd_vel_nav, through mode_mux, so it only reaches the wheels in AUTO.
"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(share('amr_control'), 'config', 'nav2.yaml')
    t = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    names = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    remap = [('cmd_vel', 'cmd_vel_nav')]
    bt = os.path.join(share('nav2_bt_navigator'), 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='log',
             parameters=[os.path.join(share('amr_control'), 'config', 'slam.yaml'), t]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='log', parameters=[params, t]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='log',
             parameters=[params, t], remappings=remap),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='log',
             parameters=[params, t], remappings=remap),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='log',
             parameters=[params, t, {'default_nav_to_pose_bt_xml': bt}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='log', parameters=[t, {'autostart': True, 'node_names': names, 'bond_timeout': 20.0}]),
    ])
