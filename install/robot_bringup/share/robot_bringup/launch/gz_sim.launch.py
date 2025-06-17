#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='empty.sdf')
    
    # Package directories
    robot_description_dir = get_package_share_directory('robot_description')
    
    # Set Gazebo resource path
    set_gazebo_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.pathsep.join([
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
            robot_description_dir
        ])
    )
    
    # Robot description (reuse the working setup)
    urdf_file = os.path.join(robot_description_dir, 'urdf', 'robot.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Start Gazebo first (empty world)
    start_gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '-v', '2'],
        output='screen',
        name='gazebo_server'
    )
    
    # Start Gazebo GUI after server is ready
    start_gazebo_client = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="Starting Gazebo GUI..."),
            ExecuteProcess(
                cmd=['ign', 'gazebo', '-g'],
                output='screen',
                name='gazebo_client'
            )
        ]
    )
    
    # Robot state publisher (using the working version)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot after Gazebo is fully ready
    spawn_robot = TimerAction(
        period=8.0,  # Give Gazebo more time to start
        actions=[
            LogInfo(msg="Spawning robot in Gazebo..."),
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'cropmap_v0',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
                ]
            )
        ]
    )
    
    # Parameter bridges (minimal set first)
    parameter_bridge = TimerAction(
        period=12.0,  # After robot is spawned
        actions=[
            LogInfo(msg="Starting parameter bridges..."),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='parameter_bridge',
                output='screen',
                arguments=[
                    # Essential topics only
                    '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # RViz (start after everything else is working)
    rviz = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="Starting RViz..."),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(robot_description_dir, 'rviz', 'robot.rviz')],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        
        # Environment
        set_gazebo_resource_path,
        
        # Log start
        LogInfo(msg="Starting CropMap V0 Gazebo simulation..."),
        LogInfo(msg="Robot description is working - now testing Gazebo integration..."),
        
        # Start components with delays
        robot_state_publisher,  # Start immediately (we know this works)
        start_gazebo_server,    # Start Gazebo server
        start_gazebo_client,    # Start Gazebo GUI after 3s
        spawn_robot,            # Spawn robot after 8s
        parameter_bridge,       # Start bridges after 12s  
        rviz                    # Start RViz after 15s
    ])

