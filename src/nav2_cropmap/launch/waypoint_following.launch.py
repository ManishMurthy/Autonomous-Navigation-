import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include the navigation launch file
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_cropmap'),
                'launch',
                'navigation.launch.py'
            ])
        ])
    )
    
    # Create a basic waypoint follower node
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'loop_rate': 20,
            'stop_on_failure': False,
            'waypoint_task_executor_plugin': 'wait_at_waypoint',
            'wait_at_waypoint': {
                'plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
                'enabled': True,
                'waypoint_pause_duration': 2
            }
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the actions
    ld.add_action(nav_launch)
    ld.add_action(waypoint_follower)
    
    return ld
