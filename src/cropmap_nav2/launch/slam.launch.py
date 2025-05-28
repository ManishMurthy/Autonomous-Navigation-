#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Get package directories
    robot_description_dir = get_package_share_directory('robot_description')
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    cropmap_nav2_dir = get_package_share_directory('cropmap_nav2')
    
    # Launch configuration variables
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            cropmap_nav2_dir,
            'config',
            'mapper_params_online_async.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for SLAM'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Using False like your working robot setup
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Launch your working robot setup - using existing launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'mini.launch.py'  # Changed to existing file
            ])
        ])
    )
    
    # Start SLAM Toolbox
    slam_toolbox_node = TimerAction(
        period=3.0,  # Wait for robot to be ready
        actions=[
            Node(
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen'
            )
        ]
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add robot and SLAM
    ld.add_action(robot_launch)
    ld.add_action(slam_toolbox_node)
    
    return ld
