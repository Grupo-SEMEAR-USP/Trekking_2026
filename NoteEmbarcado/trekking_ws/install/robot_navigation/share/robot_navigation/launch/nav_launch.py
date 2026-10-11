import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_nav = get_package_share_directory('margarete_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_nav, 'maps', 'pista_trekking.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'false'
        }.items()
    )

    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0.24', '0.0', '0.04', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
    )

    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.165', '0.0', '0.13', '0.0', '0.0', '0.0', 'base_link', 'camera_imu_link']
    )

    return LaunchDescription([
        tf_lidar,
        tf_camera,
        nav2_launch
    ])