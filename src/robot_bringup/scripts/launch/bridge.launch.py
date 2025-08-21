from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Create a bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            # Command velocity bridge (ROS 2 → Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # Odometry bridge (Gazebo → ROS 2)
            '/model/cropMap_urdf/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # Camera bridges (Gazebo → ROS 2)
            '/front_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/back_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # Clock bridge (Gazebo → ROS 2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        bridge,
    ])
