from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    robot_control_pkg_share = FindPackageShare('robot_control').find('robot_control')
    bringup_pkg_share = FindPackageShare('robot_bringup').find('robot_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    controller_config = os.path.join(robot_control_pkg_share, 'config', 'controller_config.yaml')
    world_file = os.path.join(bringup_pkg_share, 'worlds', 'farm_world.world')
    
    # Get robot description from xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            xacro_file,
            ' use_sim:=true'
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Spawn the robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'cropMap_urdf',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Include the bridge launch file
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg_share, 'launch', 'bridge.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Diff drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge_launch,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
