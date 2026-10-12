import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')
    robot_desc_pkg = get_package_share_directory('robot_description')
    trekking_gazebo_pkg = get_package_share_directory('trekking_gazebo')
    
    # Files
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(trekking_gazebo_pkg, 'worlds', 'trekking.sdf') # assuming trekking.sdf exists
    
    # Launch arguments
    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file to load into Gazebo'
    )
    
    # Environment variables
    set_env_vars = launch.actions.AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('robot_description'), '..') # to resolve package://robot_description
    )


    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 --render-engine ogre ', world]}.items()
    )

    
    # Robot State Publisher
    from launch_ros.parameter_descriptions import ParameterValue
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True
        }]
    )
    
    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'Margarete',
            '-x', '0.0',
            '-y', '13.80',
            '-z', '0.06'
        ],
        output='screen'
    )

    # ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/Margarete/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/Margarete/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/Margarete/realsense/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/Margarete/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/Margarete/realsense/imu', '/camera/imu/data'),
            ('/Margarete/cmd_vel', '/cmd_vel'),
            ('/Margarete/odom', '/odom'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        set_env_vars,
        declare_world_cmd,
        gazebo,
        robot_state_publisher,
        spawn,
        bridge
    ])

