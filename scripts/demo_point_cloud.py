#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math

class WaveletPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('demo_point_cloud')
        self.publisher_ = self.create_publisher(PointCloud2, '/simulated_point_cloud', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.t = 0.0
        self.get_logger().info('Wavelet PointCloud publisher node started, publishing on topic "/simulated_point_cloud" at 10Hz')

    def timer_callback(self):
        # Grid parameters: 10 x 20
        # A wavelet simulated refreshed at 10Hz
        points = []
        width = 10
        height = 20
        
        # Grid spacing
        dx = 0.1
        dy = 0.1
        
        # Simulating a 2D wavelet: z = A * cos(k * r - omega * t) * exp(-sigma * r^2)
        # Center the wavelet in the grid
        cx = (width - 1) * dx / 2.0
        cy = (height - 1) * dy / 2.0
        
        for i in range(width):
            for j in range(height):
                x = i * dx - cx
                y = j * dy - cy
                r = math.sqrt(x**2 + y**2)
                
                # Wavelet parameters:
                # amplitude A = 0.2
                # wave number k = 2.0 * pi / wavelength, let wavelength = 0.5 -> k = 4.0 * pi
                # angular frequency omega = 2.0 * pi * frequency, let frequency = 0.5Hz -> omega = pi
                # Gaussian decay sigma = 2.5
                z = 0.2 * math.cos(4.0 * math.pi * r - math.pi * self.t) * math.exp(-2.5 * r**2)
                
                points.append([x, y, z])
                
        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        
        # Use sensor_msgs_py.point_cloud2 to create the point cloud from points
        pc_msg = pc2.create_cloud_xyz32(header, points)
        
        self.publisher_.publish(pc_msg)
        self.t += 0.1 # 10Hz step
        
def main(args=None):
    rclpy.init(args=args)
    node = WaveletPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
