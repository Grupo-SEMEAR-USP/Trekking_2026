import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')

    world_arg = DeclareLaunchArgument('world', default_value='world', description='Nome do arquivo de mundo')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Habilita o RViz')

    cam_type_arg = DeclareLaunchArgument('cam_type', default_value='realsense', description='Câmera a ser utilizado')
    cam_type = LaunchConfiguration('cam_type')

    spawn_robot_launch_path = os.path.join(
        pkg_robot_gazebo,
        'launch',
        'spawn_robot.launch.py'
    )

    spawn_robot_node = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(spawn_robot_launch_path),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'namespace': 'Margarete',
            'x': '0.0',
            'y': '0.0',
            'z': '1.0'
        }.items()
    )

    visao_node = Node(

        package='robot_gazebo',         
        executable='vision_node.py',    
        name='vision',
        parameters=[{
            'cam_type': cam_type
        }]
    )

    return LaunchDescription([
        world_arg,
        rviz_arg,
        spawn_robot_node,
        cam_type,
        visao_node
    ])