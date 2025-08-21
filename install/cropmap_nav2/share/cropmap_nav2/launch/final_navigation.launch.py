#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Package directories
    robot_bringup_dir = FindPackageShare('robot_bringup')
    cropmap_nav2_dir = FindPackageShare('cropmap_nav2')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true')
        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([cropmap_nav2_dir, 'maps', 'final_slam_map.yaml']),
        description='Full path to saved SLAM map yaml file')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([cropmap_nav2_dir, 'config', 'nav2_params_2D.yaml']),
        description='Full path to nav2 parameters file')
        
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    # Configure parameters for nav2
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    # Robot simulation - same as before
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([robot_bringup_dir, 'launch', 'gazebo_sim.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Map Server - loads your saved SLAM map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )

    # AMCL - localization against saved map (instead of SLAM)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 Lifecycle Manager - includes map_server and amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',      # Load saved map
                'amcl',           # Localization
                'controller_server',
                'planner_server', 
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params]
    )

    # Planner Server  
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen', 
        parameters=[configured_params]
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower', 
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params]
    )

    # Safety Monitor - same as before
    safety_monitor = Node(
        package='cropmap_nav2',
        executable='safety_monitor.py',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'proximity_threshold': 0.5,
            'max_velocity': 2.0,
            'control_rate': 10.0,
            'map_resolution': 0.05,
            'map_width': 400,
            'map_height': 400,
            'height_min': -0.2,
            'height_max': 2.0,
            'robot_radius': 0.6
        }]
    )

    # RViz - same configuration
    rviz_config_file = PathJoinSubstitution([cropmap_nav2_dir, 'rviz2', 'carter_navigation.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Group Nav2 nodes
    nav2_group = GroupAction([
        lifecycle_manager,
        map_server,      # NEW: Load saved map
        amcl,           # NEW: Localization instead of SLAM
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
    ])

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        
        # Launch order
        robot_simulation,
        nav2_group,
        safety_monitor,
        rviz_node,
    ])
