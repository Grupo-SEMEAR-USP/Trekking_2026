import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')
    default_params = os.path.join(pkg_nav, 'config', 'waypoints.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='YAML com waypoints e ganhos'),
        Node(
            package='robot_navigation',
            executable='waypoint_navigator.py',
            name='waypoint_navigator',
            parameters=[params_file],
            output='screen',
        ),
    ])
