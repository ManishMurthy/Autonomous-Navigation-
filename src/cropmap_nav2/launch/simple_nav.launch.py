import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description_dir = get_package_share_directory('robot_description')
    cropmap_nav2_dir = get_package_share_directory('cropmap_nav2')
    
    # Robot State Publisher
    urdf_file = os.path.join(robot_description_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
