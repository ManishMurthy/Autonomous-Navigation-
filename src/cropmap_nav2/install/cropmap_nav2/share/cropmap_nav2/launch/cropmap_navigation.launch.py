#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    from ament_index_python.packages import get_package_share_directory
    cropmap_nav2_dir = get_package_share_directory('cropmap_nav2')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation clock'
    )
    
    # Fixed map path - use absolute path to avoid issues
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/manish/eiratech_ws/install/cropmap_nav2/share/cropmap_nav2/maps/my_field_map.yaml',
        description='Full path to map file'
    )
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(cropmap_nav2_dir, 'config', 'nav_t_params.yaml'),
        description='Full path to Nav2 parameters'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true', 
        description='Automatically startup nav2'
    )
    
    # KEEP this for initial localization - AMCL needs a starting point
    # This will be overridden by AMCL once localization starts
    # Horizontal +90 degree rotation to match Gazebo orientation
    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '0', 'map', 'odom'],  # +90 degrees around X-axis (horizontal)
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add missing base_footprint to base_link transform to fix TF tree
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Use navigation launch only (more stable than bringup)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'  
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'autostart': autostart
        }.items()
    )
    
    # Localization launch separately for better control
    localization_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'autostart': autostart
        }.items()
    )
    
    # RViz
    rviz_config_file = os.path.join(cropmap_nav2_dir, 'rviz2', 'carter_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add nodes in order
    ld.add_action(static_transform_map_odom)     # Provides initial map frame
    ld.add_action(base_footprint_to_base_link)   # Fix TF tree connection
    ld.add_action(localization_bringup)         # Map server and AMCL
    ld.add_action(nav2_bringup)                 # Navigation components
    ld.add_action(rviz_node)                    # RViz last
    
    return ld
