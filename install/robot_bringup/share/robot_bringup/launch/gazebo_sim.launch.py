import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('robot_description')
    
    # Robot description path
    urdf_path = os.path.join(pkg_share, 'urdf', 'cropMap_urdf.urdf')
    
    # Bridge config path
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge.yaml')
    
    # Launch Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path, 'r').read(),
                    'use_sim_time': True}]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/empty/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '300',
             '--req', 'sdf_filename: "' + urdf_path + '", name: "cropmap_robot"'],
        output='screen'
    )
    
    # ROS - Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen'
    )
    
    # Teleop keyboard for testing movement
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        teleop
    ])
