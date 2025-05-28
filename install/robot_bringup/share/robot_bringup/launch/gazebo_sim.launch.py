#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    # Get package directories
    robot_description_dir = get_package_share_directory('robot_description')
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot description file path
    urdf_file = os.path.join(robot_description_dir, 'urdf', 'cropMap_urdf.urdf')
    
    # Read URDF file and wrap it properly as string parameter
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    robot_description_param = ParameterValue(robot_desc, value_type=str)
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher - ESSENTIAL for wheel transforms
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf', '-r'],
        output='screen'
    )
    
    # Spawn robot in Gazebo after delay
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'cropmap_robot',
                    '-z', '0.5'
                ],
                output='screen'
            )
        ]
    )
    
    # ROS-Gazebo Bridge - Add sensor data
    ros_gz_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                remappings=[
                    ('/lidar', '/scan'),
                ]
            )
        ]
    )
    
    # Launch RViz with proper fixed frame
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,  # This is crucial!
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        rviz_node
    ])

