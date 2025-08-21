#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    cropmap_nav2_dir = get_package_share_directory('cropmap_nav2')
    
    # World file path
    world_file = os.path.join(robot_bringup_dir, 'worlds', 'terrain_4_obstacle.world')
    
    return LaunchDescription([
        # Launch Gazebo with terrain
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_bringup_dir, '/launch/gazebo_sim.launch.py']),
            launch_arguments={'world': world_file}.items()
        ),
        
        # Launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cropmap_nav2_dir, '/launch/cropmap_navigation.launch.py'])
        ),
        
        # Start terrain test node
        Node(
            package='cropmap_nav2',
            executable='terrain_test_node',
            name='terrain_test',
            parameters=[{'terrain_type': 'terrain_4_obstacle'}]
        )
    ])
