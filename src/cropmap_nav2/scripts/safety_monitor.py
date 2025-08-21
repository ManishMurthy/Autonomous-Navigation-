#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
import tf2_ros
from tf_transformations import euler_from_quaternion

# ROS2 Messages
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, Bool, Float32
from std_srvs.srv import SetBool
import sensor_msgs_py.point_cloud2 as pc2

class SafetyMonitor(Node):
    """
    Safety Monitor for Nav2 with 3D to 2D SLAM Conversion
    
    Features:
    - Converts 3D LiDAR data to 2D occupancy grids for SLAM
    - Monitors Nav2 navigation safety per acceptance criteria
    - Emergency stop functionality
    - Velocity limiting and proximity detection
    - Compatible with existing Nav2 setup
    """
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Parameters from your launch file - use declare_parameter with fallback
        self.declare_parameter('proximity_threshold', 0.5)  # m - Acceptance criteria
        self.declare_parameter('max_velocity', 2.0)         # m/s - Acceptance criteria  
        self.declare_parameter('control_rate', 10.0)        # Hz
        
        # Additional parameters for 3D to 2D conversion
        self.declare_parameter('map_resolution', 0.05)      # m/pixel
        self.declare_parameter('map_width', 400)            # pixels (20m field)
        self.declare_parameter('map_height', 400)           # pixels
        self.declare_parameter('height_min', -0.2)          # m - Ground level
        self.declare_parameter('height_max', 2.0)           # m - Max obstacle height
        self.declare_parameter('robot_radius', 0.6)         # m - CropMap V0 footprint
        
        # Get parameters (use_sim_time may already be declared by launch file)
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        except:
            self.declare_parameter('use_sim_time', True)
            self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
            
        self.proximity_threshold = self.get_parameter('proximity_threshold').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.height_min = self.get_parameter('height_min').get_parameter_value().double_value
        self.height_max = self.get_parameter('height_max').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        
        # Safety state
        self.emergency_stop = False
        self.proximity_alert = False
        self.current_velocity = 0.0
        self.robot_pose = None
        self.last_cmd_vel = Twist()
        
        # 3D to 2D mapping
        self.occupancy_grid = np.full((self.map_height, self.map_width), 0, dtype=np.int8)  # Free space
        self.map_origin_x = -10.0  # Center 20x20m map
        self.map_origin_y = -10.0
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            self.qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            self.qos_profile
        )
        
        # 3D LiDAR data for SLAM conversion
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',  # Your CropMap V0 LiDAR
            self.lidar_callback,
            self.qos_profile
        )
        
        # Publishers
        # SLAM outputs
        self.slam_map_pub = self.create_publisher(
            OccupancyGrid,
            '/map_from_lidar',
            self.qos_profile
        )
        
        self.slam_scan_pub = self.create_publisher(
            LaserScan,
            '/scan_from_lidar',
            self.qos_profile
        )
        
        # Safety status
        self.proximity_pub = self.create_publisher(
            Bool,
            '/safety/proximity_alert',
            self.qos_profile
        )
        
        self.emergency_pub = self.create_publisher(
            Bool,
            '/safety/emergency_stop',
            self.qos_profile
        )
        
        self.velocity_pub = self.create_publisher(
            Float32,
            '/safety/current_velocity',
            self.qos_profile
        )
        
        # Safe command output
        self.safe_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            self.qos_profile
        )
        
        # Services
        self.emergency_service = self.create_service(
            SetBool,
            '/safety/emergency_stop_toggle',
            self.emergency_stop_callback
        )
        
        # Timers
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        self.slam_timer = self.create_timer(
            1.0,  # Publish SLAM data at 1 Hz
            self.publish_slam_data
        )
        
        self.get_logger().info("Safety Monitor initialized for Nav2 + SLAM")
        self.get_logger().info(f"Max velocity: {self.max_velocity} m/s")
        self.get_logger().info(f"Proximity threshold: {self.proximity_threshold} m")
        self.get_logger().info(f"Publishing 3D→2D SLAM data")
        
    def lidar_callback(self, msg):
        """Convert 3D LiDAR to 2D data for SLAM and check safety"""
        try:
            # Extract 3D points
            points_3d = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point
                
                # Filter by height (ground vs obstacles)
                if self.height_min <= z <= self.height_max:
                    points_3d.append([x, y, z])
            
            if not points_3d:
                return
                
            points_3d = np.array(points_3d)
            
            # Update occupancy grid for SLAM
            self.update_occupancy_grid(points_3d)
            
            # Create 2D laser scan for SLAM
            self.create_laser_scan(points_3d, msg.header)
            
            # Safety proximity check
            self.check_proximity_safety(points_3d)
            
        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR: {e}")
    
    def update_occupancy_grid(self, points_3d):
        """Update 2D occupancy grid from 3D points"""
        # Reset grid to free space
        self.occupancy_grid.fill(0)
        
        if self.robot_pose is None:
            return
            
        # Get robot position
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        
        # Convert robot-relative points to world coordinates
        world_points = points_3d.copy()
        world_points[:, 0] += robot_x
        world_points[:, 1] += robot_y
        
        # Convert to grid coordinates
        grid_x = ((world_points[:, 0] - self.map_origin_x) / self.map_resolution).astype(int)
        grid_y = ((world_points[:, 1] - self.map_origin_y) / self.map_resolution).astype(int)
        
        # Filter valid coordinates
        valid_mask = (
            (grid_x >= 0) & (grid_x < self.map_width) &
            (grid_y >= 0) & (grid_y < self.map_height)
        )
        
        valid_x = grid_x[valid_mask]
        valid_y = grid_y[valid_mask]
        valid_z = world_points[valid_mask, 2]
        
        # Mark obstacles
        for x, y, z in zip(valid_x, valid_y, valid_z):
            if z > 0.1:  # Above ground
                self.occupancy_grid[y, x] = 100  # Occupied
    
    def create_laser_scan(self, points_3d, header):
        """Create 2D laser scan from 3D points for SLAM"""
        scan = LaserScan()
        scan.header.stamp = header.stamp
        scan.header.frame_id = 'base_link'
        
        # 360-degree scan parameters
        scan.angle_min = -np.pi
        scan.angle_max = np.pi
        scan.angle_increment = np.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 15.0  # Livox MID-360 range
        
        # Initialize ranges
        num_rays = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        ranges = np.full(num_rays, scan.range_max)
        
        # Project 3D points to 2D
        for point in points_3d:
            x, y, z = point
            
            # Skip ground points
            if z < 0.1:
                continue
                
            # Calculate polar coordinates
            angle = np.arctan2(y, x)
            range_val = np.sqrt(x*x + y*y)
            
            # Find ray index
            ray_index = int((angle - scan.angle_min) / scan.angle_increment)
            
            if 0 <= ray_index < num_rays:
                # Keep closest obstacle per ray
                if range_val < ranges[ray_index]:
                    ranges[ray_index] = range_val
        
        scan.ranges = ranges.tolist()
        scan.intensities = []
        
        self.slam_scan_pub.publish(scan)
    
    def check_proximity_safety(self, points_3d):
        """Check proximity for safety (acceptance criteria)"""
        if len(points_3d) == 0:
            return
            
        # Calculate distances to obstacles
        distances = np.sqrt(points_3d[:, 0]**2 + points_3d[:, 1]**2)
        min_distance = np.min(distances) if len(distances) > 0 else float('inf')
        
        # Update proximity alert
        previous_alert = self.proximity_alert
        self.proximity_alert = min_distance < self.proximity_threshold
        
        # Log changes
        if self.proximity_alert and not previous_alert:
            self.get_logger().warn(f"Proximity alert! Obstacle at {min_distance:.2f}m")
        elif not self.proximity_alert and previous_alert:
            self.get_logger().info("Proximity alert cleared")
        
        # Emergency stop if too close (acceptance criteria)
        if min_distance < (self.proximity_threshold * 0.8):
            if not self.emergency_stop:
                self.emergency_stop = True
                self.get_logger().error(f"EMERGENCY STOP! Obstacle at {min_distance:.2f}m")
    
    def cmd_vel_callback(self, msg):
        """Monitor command velocity"""
        self.last_cmd_vel = msg
        
        # Check velocity limits
        linear_speed = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
        if linear_speed > self.max_velocity:
            self.get_logger().warn(f"Velocity exceeds limit: {linear_speed:.2f} > {self.max_velocity}")
    
    def odometry_callback(self, msg):
        """Update robot state"""
        self.robot_pose = msg.pose.pose
        
        # Calculate current velocity
        linear_vel = msg.twist.twist.linear
        self.current_velocity = np.sqrt(linear_vel.x**2 + linear_vel.y**2)
    
    def control_loop(self):
        """Main safety control loop"""
        # Publish safety status
        self.proximity_pub.publish(Bool(data=self.proximity_alert))
        self.emergency_pub.publish(Bool(data=self.emergency_stop))
        self.velocity_pub.publish(Float32(data=self.current_velocity))
        
        # Create safe command
        safe_cmd = Twist()
        
        if self.emergency_stop:
            # Complete stop (acceptance criteria)
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.angular.z = 0.0
        else:
            # Apply velocity limits
            safe_cmd = self.apply_velocity_limits(self.last_cmd_vel)
        
        # Publish safe command
        self.safe_cmd_pub.publish(safe_cmd)
    
    def apply_velocity_limits(self, cmd_vel):
        """Apply safety velocity limits (acceptance criteria)"""
        safe_cmd = Twist()
        
        # Calculate total linear velocity
        total_linear = np.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)
        
        # Apply max velocity limit
        if total_linear > self.max_velocity:
            scale = self.max_velocity / total_linear
            safe_cmd.linear.x = cmd_vel.linear.x * scale
            safe_cmd.linear.y = cmd_vel.linear.y * scale
        else:
            safe_cmd.linear.x = cmd_vel.linear.x
            safe_cmd.linear.y = cmd_vel.linear.y
        
        safe_cmd.linear.z = 0.0
        safe_cmd.angular = cmd_vel.angular
        
        # Reduce speed near obstacles
        if self.proximity_alert:
            safety_scale = 0.3  # 30% speed near obstacles
            safe_cmd.linear.x *= safety_scale
            safe_cmd.linear.y *= safety_scale
            safe_cmd.angular.z *= safety_scale
        
        return safe_cmd
    
    def publish_slam_data(self):
        """Publish occupancy grid for SLAM"""
        if self.robot_pose is None:
            return
            
        # Create occupancy grid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        # Map metadata
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Map data
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.slam_map_pub.publish(grid_msg)
    
    def emergency_stop_callback(self, request, response):
        """Emergency stop service (acceptance criteria)"""
        self.emergency_stop = request.data
        
        if self.emergency_stop:
            self.get_logger().warn("Emergency stop ACTIVATED")
        else:
            self.get_logger().info("Emergency stop DEACTIVATED")
        
        response.success = True
        response.message = f"Emergency stop {'activated' if self.emergency_stop else 'deactivated'}"
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
