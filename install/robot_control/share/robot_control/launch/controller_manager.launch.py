from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    # Controller configuration file
    controller_config = PathJoinSubstitution([
        FindPackageShare('robot_control'),
        'config',
        'controller_config.yaml'
    ])
    
    # Controller manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        remappings=[
            ('~/cmd_vel', '/cmd_vel'),
        ]
    )
    
    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Spawn diff drive controller (with delay)
    diff_drive_controller_spawner = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])
