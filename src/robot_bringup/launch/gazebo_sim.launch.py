#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_bringup'),
            'worlds',
            'field_obstacles.world'
        ]),
        description='Full path to world file'
    )

    # Get robot description file path
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'robot.urdf.xacro'
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', robot_description_path]),
                value_type=str
            ),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot entity
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'cropmap_v0',
            '-x', '10.0',
            '-y', '10.0', 
            '-z', '1.0'
        ]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'maps',
                'my_field_map.yaml'
            ])
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        joint_state_publisher,
        map_server_node
    ])
