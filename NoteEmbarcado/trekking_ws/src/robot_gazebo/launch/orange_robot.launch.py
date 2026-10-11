"""Uma execução: visão laranja, RPLidar A2M12, missão e base (conforme o modo)."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def assemble(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode not in ('preview','align','mission'):
        raise ValueError('mode deve ser preview, align ou mission')
    count = int(arg('cones'))
    if not 1 <= count <= 4:
        raise ValueError('cones deve ficar entre 1 e 4')
    if mode != 'preview' and arg('camera') != 'realsense':
        raise ValueError('Movimento exige RealSense; cv2 é apenas para preview')
    config = arg('params_file')
    vision = Node(package='robot_gazebo',executable='vision_node.py',name='vision',output='screen',
                  parameters=[config,{'cam_type':arg('camera'),'show_image':arg('show_image').lower() == 'true'}])
    mission = Node(package='robot_gazebo',executable='orange_mission_node.py',name='orange_mission',output='screen',
                   parameters=[config,{'mode':mode,'max_cones':count,'auto_start':True}])
    nodes = [vision,mission]
    if arg('start_lidar').lower() == 'true':
        nodes.append(Node(package='sllidar_ros2',executable='sllidar_node',name='sllidar_node',output='screen',
                          parameters=[{'channel_type':'serial','serial_port':arg('lidar_port'),
                                       'serial_baudrate':256000,'frame_id':'laser',
                                       'inverted':False,'angle_compensate':True,'scan_mode':'Sensitivity'}]))
    if mode != 'preview':
        if os.path.realpath(arg('esp_port')) == os.path.realpath(arg('lidar_port')) and arg('start_lidar').lower() == 'true':
            raise ValueError('ESP e LiDAR não podem usar a mesma porta serial')
        nodes += [
            Node(package='robot_gazebo',executable='uart_comm.py',name='uart_comm',output='screen',
                 parameters=[config,{'port':arg('esp_port')}]),
            Node(package='robot_gazebo',executable='twist2ackermann.py',name='twist2ackermann',output='screen',parameters=[config]),
            Node(package='robot_gazebo',executable='odom_node.py',name='odom_publisher',output='screen',parameters=[config])]
    # Uma falha de processo encerra todos; IDs não são reutilizados silenciosamente após reinício da visão.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason='Um processo do robô encerrou; parando a execução.'))]))
                for node in nodes]
    return handlers+nodes


def generate_launch_description():
    share = get_package_share_directory('robot_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('mode',default_value='preview',description='preview, align ou mission'),
        DeclareLaunchArgument('cones',default_value='4',description='1 para ensaio de chegada; 4 para missão'),
        DeclareLaunchArgument('esp_port',default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_port',default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('start_lidar',default_value='true',description='false se o driver já publica /scan'),
        DeclareLaunchArgument('camera',default_value='realsense'),
        DeclareLaunchArgument('show_image',default_value='false'),
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','orange_robot.yaml')),
        OpaqueFunction(function=assemble)])
