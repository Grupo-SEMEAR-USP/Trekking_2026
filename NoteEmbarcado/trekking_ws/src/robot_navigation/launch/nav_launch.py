import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg=get_package_share_directory('robot_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Caminho absoluto de um mapa YAML existente'),
        DeclareLaunchArgument('params_file',default_value=os.path.join(pkg,'config','nav2_params.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),'launch','bringup_launch.py')),
            launch_arguments={'map':LaunchConfiguration('map'),
                              'params_file':LaunchConfiguration('params_file'),
                              'use_sim_time':'false'}.items())
    ])
