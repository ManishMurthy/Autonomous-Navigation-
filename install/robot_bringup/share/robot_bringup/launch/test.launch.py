from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Gazebo only
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v4'],  # Verbose mode to see more details
        output='screen'
    )
    
    # Test publisher to send movement commands
    test_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r', '10', '/cmd_vel', 'geometry_msgs/msg/Twist',
             '"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        test_publisher
    ])
