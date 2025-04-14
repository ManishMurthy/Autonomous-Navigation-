from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    robot_description_pkg = FindPackageShare('robot_description')
    robot_control_pkg = FindPackageShare('robot_control')
    
    # Controller parameters
    controller_params_file = PathJoinSubstitution([robot_control_pkg, 'config', 'controller_config.yaml'])
    
    # Start the controller manager with the joint state broadcaster and diff drive controller
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([robot_description_pkg, 'urdf', 'robot.urdf.xacro'])
            ])},
            controller_params_file
        ],
        output='screen',
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # Diff drive controller spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )
    
    # Make sure the joint state broadcaster is started before the diff drive controller
    diff_drive_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    # Create and return the launch description
    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_delay,
    ])
