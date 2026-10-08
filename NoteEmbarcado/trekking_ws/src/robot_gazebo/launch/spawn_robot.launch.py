import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):

    ns = LaunchConfiguration('namespace').perform(context)
    world = LaunchConfiguration('world').perform(context)

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_robot_description = get_package_share_directory('robot_description')

    world_path = os.path.join(pkg_robot_gazebo, 'worlds', f'{world}.sdf')
    
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    gz_flags = '-s -r' if headless else '-r'

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{gz_flags} {world_path}'}.items(),
    )

    robot_xacro = os.path.join(
        pkg_robot_description,
        'urdf',
        'robot.urdf.xacro'
    )

    sensors_xacro_path = os.path.join(pkg_robot_gazebo, 'config', 'sensors.xacro')

    robot_cmd = [
        'xacro', 
        robot_xacro,
        f'sensors_config:={sensors_xacro_path}',
        f'namespace:={ns}'
    ]
    urdf_str = subprocess.run(
        robot_cmd, check=True, capture_output=True, text=True).stdout

    with tempfile.NamedTemporaryFile(
            'w', suffix=f'_{ns}.urdf', delete=False) as f:
        f.write(urdf_str)
        urdf_path = f.name
    sdf_str = subprocess.run(
        ['gz', 'sdf', '-p', urdf_path],
        check=True, capture_output=True, text=True).stdout

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz_enabled = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1')
    if rviz_enabled:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', os.path.join(pkg_robot_gazebo, 'config', 'rviz.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

    teleop_enabled = LaunchConfiguration('teleop').perform(context).lower() in ('true', '1')
    if teleop_enabled:
        teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='gnome-terminal --',
            remappings=[('cmd_vel', f'/{ns}/cmd_vel')],
            output='screen'
        )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        namespace=ns,
        arguments=[
            '-string', sdf_str,
            '-name', ns,
            '-allow_renaming', 'false',
            '-x', x, '-y', y, '-z', '1.0', '-Y', yaw
        ],
        output='screen'
    )

    robot_description = ParameterValue(urdf_str, value_type=str)

    imu_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_bridge',
        namespace=ns,
        arguments=[
            '0', '0', '0', '0', '0', '0', 
            'base_link',  
            'imu_link'    
        ],
        parameters=[{'use_sim_time': True}],   
        remappings=[
            ('/tf', '/tf'), 
            ('/tf_static', '/tf_static')
        ],
        output='screen'
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'robot_description': robot_description, 
            'use_sim_time': True
        }],
        remappings=[
            ('/joint_states', f'/{ns}/joint_states'),
            ('/tf', '/tf'),                   
            ('/tf_static', '/tf_static')      
        ],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=ns,
        arguments=[
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[

        ],
        output='screen'
    )

    actions = [gz_sim_cmd, clock_bridge, spawn_node, imu_tf_bridge, rsp_node, bridge_node]

    if rviz_enabled:
        actions.append(rviz_node)

    if teleop_enabled:
        actions.append(teleop_node)

    return actions

def generate_launch_description():

    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_robot_desc = get_package_share_directory('robot_description')

    robot_gazebo_share_parent = os.path.dirname(pkg_robot_gazebo)
    robot_desc_share = os.path.dirname(pkg_robot_desc)

    model_paths = f"{robot_gazebo_share_parent}:{robot_desc_share}"

    set_gz_path_cmd = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_paths
    )

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),
        
        DeclareLaunchArgument('world', default_value='', description='nome do mundo'),
        DeclareLaunchArgument('namespace', default_value='Margarete', description='nome do barco'),

        DeclareLaunchArgument('headless', default_value='false', description='true roda o Gazebo sem interface grafica'),
        DeclareLaunchArgument('rviz', default_value='true', description='false desabilita o RViz'),
        DeclareLaunchArgument('teleop', default_value='true', description='false desabilita o controle teleop da margs'),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        set_gz_path_cmd,

        OpaqueFunction(function=launch_setup)
    ])