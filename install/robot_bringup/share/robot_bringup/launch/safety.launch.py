#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='farm_world.world',
        description='Gazebo world file'
    )
    
    # Get package directories
    robot_bringup_dir = FindPackageShare('robot_bringup')
    cropmap_nav2_dir = FindPackageShare('cropmap_nav2')
    
    # Include base simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_bringup_dir, 'launch', 'gazebo_sim.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    # Safety Monitor Node
    safety_monitor = Node(
        package='cropmap_nav2',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'proximity_threshold': 0.5,  # Acceptance criteria: 0.5m auto-stop
            'max_velocity': 2.0,          # Acceptance criteria: max 2.0 m/s
            'control_rate': 10.0          # 10 Hz control loop
        }],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_nav'),  # Raw commands from navigation
            ('/cmd_vel', '/cmd_vel'),          # Safe commands to robot
        ],
        output='screen'
    )
    
    # Velocity Command Remapping Node (routes navigation through safety monitor)
    cmd_vel_remapper = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_remapper',
        arguments=['/cmd_vel_nav', '/cmd_vel_raw'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Emergency Stop Service Client (for testing)
    emergency_stop_client = Node(
        package='cropmap_nav2',
        executable='emergency_stop_client.py',
        name='emergency_stop_client',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Group all safety components
    safety_group = GroupAction([
        safety_monitor,
        cmd_vel_remapper,
        # emergency_stop_client,  # Uncomment when you create the client script
    ])
    
    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        gazebo_launch,
        safety_group,
    ])
