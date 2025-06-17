# Add this to your Nav2 launch file:

# Auto initial pose publisher
initial_pose_publisher = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='initial_pose_publisher',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'autostart': True,
        'node_names': []  # Empty, just for the publisher
    }]
)

# Alternative: Use a simple publisher node
initial_pose_cmd = ExecuteProcess(
    cmd=[
        'ros2', 'topic', 'pub', '--once',
        '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
        '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, '
        'pose: {pose: {position: {x: 10.0, y: 10.0, z: 0.0}, '
        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
        'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
        '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
        '0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}'
    ],
    output='screen'
)

# Add delay and then publish initial pose
delayed_initial_pose = TimerAction(
    period=5.0,  # Wait 5 seconds after launch
    actions=[initial_pose_cmd]
)
