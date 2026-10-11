"""Controle real. Gazebo e Nav2 não são iniciados junto ao centralizador."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=get_package_share_directory('robot_gazebo')
    config=LaunchConfiguration('params_file')
    port=LaunchConfiguration('port')
    camera=LaunchConfiguration('camera')
    nodes=[
        Node(package='robot_gazebo',executable='uart_comm.py',name='uart_comm',
             parameters=[config,{'port':port}],output='screen'),
        Node(package='robot_gazebo',executable='twist2ackermann.py',name='twist2ackermann',
             parameters=[config],output='screen'),
        Node(package='robot_gazebo',executable='odom_node.py',name='odom_publisher',
             parameters=[config],output='screen'),
        Node(package='robot_gazebo',executable='vision_node.py',name='vision',
             remappings=[('orange_cone_center','cone_center')],
             parameters=[config],condition=IfCondition(camera),output='screen'),
        Node(package='robot_gazebo',executable='centraliza_cone.py',name='centraliza_cone',
             parameters=[config],condition=IfCondition(LaunchConfiguration('centering')),output='screen')
    ]
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','real_robot.yaml')),
        DeclareLaunchArgument('port',default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('camera',default_value='true'),
        DeclareLaunchArgument('centering',default_value='true'),
        *nodes
    ])
