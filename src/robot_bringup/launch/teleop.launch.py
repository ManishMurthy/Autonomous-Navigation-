from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    robot_control_pkg_share = FindPackageShare('robot_control').find('robot_control')
    
    # Paths
    teleop_config = PathJoinSubstitution([robot_control_pkg_share, 'config', 'teleop_config.yaml'])
    
    # Teleop twist keyboard node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )
    
    return LaunchDescription([
        teleop_node
    ])
