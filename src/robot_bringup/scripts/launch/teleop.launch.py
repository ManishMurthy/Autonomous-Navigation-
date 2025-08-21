#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    
    # Method 1: Simple teleop_twist_keyboard (if package is installed)
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='gnome-terminal --',  # Use gnome-terminal instead of xterm
        remappings=[
            ('/cmd_vel', '/cmd_vel')  # Direct mapping to your robot's cmd_vel
        ],
        parameters=[{
            'speed': 0.5,       # Linear speed
            'turn': 1.0,        # Angular speed  
            'repeat_rate': 10.0 # Publishing rate
        }]
    )
    
    # Method 2: Simple manual control via terminal commands
    info_message = LogInfo(
        msg="Teleop ready! Use WASD keys to control robot:\n"
            "W/S: Forward/Backward\n"
            "A/D: Turn Left/Right\n"
            "X: Stop\n"
            "Q: Increase speed, Z: Decrease speed"
    )
    
    return LaunchDescription([
        info_message,
        teleop_keyboard
    ])
