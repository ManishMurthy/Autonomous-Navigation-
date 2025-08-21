#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import time

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        self.max_linear_velocity = 2.0
        self.emergency_stop_active = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_status_pub = self.create_publisher(Bool, '/safety_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Services
        self.emergency_stop_srv = self.create_service(Empty, '/emergency_stop', self.emergency_stop_callback)
        self.resume_srv = self.create_service(Empty, '/resume_navigation', self.resume_callback)
        
        # Timer for safety status
        self.timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info("🛡️ Safety Monitor started")

    def cmd_vel_callback(self, msg):
        if self.emergency_stop_active:
            safe_cmd = Twist()  # Zero velocity
            self.get_logger().warn("🚨 Emergency stop active")
        else:
            safe_cmd = self.apply_safety_limits(msg)
        
        self.cmd_vel_pub.publish(safe_cmd)

    def apply_safety_limits(self, cmd_vel):
        safe_cmd = Twist()
        
        # Limit linear velocity
        if abs(cmd_vel.linear.x) > self.max_linear_velocity:
            sign = 1 if cmd_vel.linear.x > 0 else -1
            safe_cmd.linear.x = sign * self.max_linear_velocity
            self.get_logger().warn(f"⚠️ Speed limited to {self.max_linear_velocity} m/s")
        else:
            safe_cmd.linear.x = cmd_vel.linear.x
        
        safe_cmd.angular.z = cmd_vel.angular.z
        return safe_cmd

    def odom_callback(self, msg):
        current_speed = abs(msg.twist.twist.linear.x)
        if current_speed > self.max_linear_velocity + 0.1:
            self.get_logger().error(f"🚨 OVER-SPEED: {current_speed:.2f} m/s")

    def emergency_stop_callback(self, request, response):
        self.emergency_stop_active = True
        self.get_logger().info("🛑 Emergency stop activated")
        return response

    def resume_callback(self, request, response):
        self.emergency_stop_active = False
        self.get_logger().info("▶️ Navigation resumed")
        return response

    def publish_status(self):
        status_msg = Bool()
        status_msg.data = not self.emergency_stop_active
        self.safety_status_pub.publish(status_msg)

def main():
    rclpy.init()
    safety_monitor = SafetyMonitor()
    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
