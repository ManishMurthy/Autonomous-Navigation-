#!/usr/bin/env python3
# cropmap_nav2/scripts/stage2_validation.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import time
import math

class Stage2Validator(Node):
    def __init__(self):
        super().__init__('stage2_validator')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_callback, 10)
        
        # Publishers for testing
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Validation tracking
        self.max_velocity = 0.0
        self.current_velocity = 0.0
        self.lidar_data_received = False
        self.navigation_active = False
        
        # Timers
        self.create_timer(1.0, self.validation_check)
        
        self.get_logger().info("Stage 2 Validator initialized")

    def cmd_vel_callback(self, msg):
        # Safety Criteria 1: Speed Limitation
        linear_speed = abs(msg.linear.x)
        if linear_speed > self.max_velocity:
            self.max_velocity = linear_speed
        
        self.current_velocity = linear_speed
        
        # Check if exceeds 2.0 m/s
        if linear_speed > 2.0:
            self.get_logger().error(f"SAFETY VIOLATION: Speed {linear_speed:.2f} m/s exceeds 2.0 m/s limit!")
            
        # Implementation Criteria: Check message format
        if msg.linear.y != 0.0 or msg.angular.x != 0.0 or msg.angular.y != 0.0:
            self.get_logger().warn("Cmd_vel has non-zero values in unused fields")

    def odom_callback(self, msg):
        self.navigation_active = True

    def lidar_callback(self, msg):
        # Implementation Criteria: LiDAR format check
        if msg.header.frame_id != "" and len(msg.data) > 0:
            self.lidar_data_received = True
        else:
            self.get_logger().warn("LiDAR data format issue detected")

    def validation_check(self):
        self.get_logger().info(f"Validation Status:")
        self.get_logger().info(f"  Max velocity recorded: {self.max_velocity:.2f} m/s")
        self.get_logger().info(f"  Current velocity: {self.current_velocity:.2f} m/s") 
        self.get_logger().info(f"  LiDAR data received: {self.lidar_data_received}")
        self.get_logger().info(f"  Navigation active: {self.navigation_active}")
        
        # Speed compliance check
        if self.max_velocity <= 2.0:
            self.get_logger().info("✅ Speed limitation compliance: PASS")
        else:
            self.get_logger().error("❌ Speed limitation compliance: FAIL")

def main():
    rclpy.init()
    validator = Stage2Validator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
