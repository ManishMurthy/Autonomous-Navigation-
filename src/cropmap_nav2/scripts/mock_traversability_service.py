#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vox_nav_msgs.srv import GetTraversabilityMap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class MockTraversabilityService(Node):
    def __init__(self):
        super().__init__('mock_traversability_service')
        
        # Create the service
        self.srv = self.create_service(
            GetTraversabilityMap, 
            'get_traversability_map', 
            self.get_traversability_callback
        )
        
        self.get_logger().info('Mock Traversability Service is ready!')
        self.get_logger().info('VoxNav can now proceed with planning')

    def get_traversability_callback(self, request, response):
        self.get_logger().info('Received traversability map request - providing mock response')
        
        # Create a minimal valid point cloud response
        response.traversability_map = PointCloud2()
        response.traversability_map.header = Header()
        response.traversability_map.header.frame_id = "map"
        response.traversability_map.header.stamp = self.get_clock().now().to_msg()
        
        # Minimal point cloud data
        response.traversability_map.height = 1
        response.traversability_map.width = 0
        response.traversability_map.fields = []
        response.traversability_map.is_bigendian = False
        response.traversability_map.point_step = 0
        response.traversability_map.row_step = 0
        response.traversability_map.data = []
        response.traversability_map.is_dense = True
        
        self.get_logger().info('Mock traversability map sent - VoxNav should proceed')
        return response

def main(args=None):
    rclpy.init(args=args)
    
    service = MockTraversabilityService()
    
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
