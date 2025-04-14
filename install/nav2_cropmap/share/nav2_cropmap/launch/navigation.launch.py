import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get the launch directory
    nav2_cropmap_dir = get_package_share_directory('nav2_cropmap')
    robot_description_dir = get_package_share_directory('robot_description')
    
    # Nav2 parameters
    nav2_params_path = os.path.join(nav2_cropmap_dir, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(nav2_cropmap_dir, 'maps', 'farm_map.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')
    
    # Start Gazebo with the farm world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'gazebo_sim.launch.py'
            ])
        ])
    )
    
    # Start Rviz2
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'rviz.launch.py'
            ])
        ])
    )
    
    # Start Nav2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path
        }.items()
    )
    
    # Create a delayed action to start Nav2 after Gazebo is fully loaded
    delayed_nav2_bringup = TimerAction(
        period=5.0,
        actions=[nav2_bringup_launch]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    
    # Add the actions
    ld.add_action(gazebo_launch)
    ld.add_action(rviz_launch)
    ld.add_action(delayed_nav2_bringup)
    
    return ld
