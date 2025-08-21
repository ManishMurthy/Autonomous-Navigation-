#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import time
import threading
import os

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        self.metrics = {
            'cmd_vel_count': 0,
            'max_speed': 0.0,
            'current_speed': 0.0,
            'safety_violations': 0,
            'start_time': time.time()
        }
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.safety_sub = self.create_subscription(Bool, '/safety_status', self.safety_callback, 10)
        
        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info("📊 Performance Monitor started")

    def cmd_vel_callback(self, msg):
        self.metrics['cmd_vel_count'] += 1
        speed = abs(msg.linear.x)
        self.metrics['current_speed'] = speed
        
        if speed > self.metrics['max_speed']:
            self.metrics['max_speed'] = speed
        
        if speed > 2.0:
            self.metrics['safety_violations'] += 1

    def odom_callback(self, msg):
        pass  # Could add odometry monitoring here

    def safety_callback(self, msg):
        pass  # Could add safety status monitoring here

    def display_loop(self):
        while True:
            try:
                os.system('clear')
                runtime = time.time() - self.metrics['start_time']
                
                print("=" * 60)
                print(f"🤖 CropMap V0 Performance Monitor")
                print("=" * 60)
                print(f"Runtime: {runtime:.1f}s")
                print(f"Commands: {self.metrics['cmd_vel_count']}")
                print(f"Current Speed: {self.metrics['current_speed']:.3f} m/s")
                print(f"Max Speed: {self.metrics['max_speed']:.3f} m/s")
                print(f"Safety Violations: {self.metrics['safety_violations']}")
                
                speed_ok = self.metrics['max_speed'] <= 2.0
                print(f"Speed Compliance: {'✅ PASS' if speed_ok else '❌ FAIL'}")
                print("=" * 60)
                
                time.sleep(1.0)
            except KeyboardInterrupt:
                break

def main():
    rclpy.init()
    monitor = PerformanceMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
