from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directories
    bringup_pkg_share = FindPackageShare('robot_bringup').find('robot_bringup')
    
    # Include launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg_share, 'launch', 'gazebo_sim.launch.py'])
        ])
    )
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg_share, 'launch', 'rviz.launch.py'])
        ])
    )
    
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg_share, 'launch', 'teleop.launch.py'])
        ])
    )
    
    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        teleop_launch
    ])
