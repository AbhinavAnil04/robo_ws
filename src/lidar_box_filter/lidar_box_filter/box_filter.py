#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class BoxFilter(Node):
    def __init__(self):
        super().__init__('lidar_box_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',            # Original LiDAR topic
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_filtered',   # Filtered LiDAR topic
            10
        )
        # Box filter bounds in robot frame (example values)
        self.x_min = -0.28
        self.x_max = 0.28
        self.y_min = -0.28
        self.y_max = 0.28

    def scan_callback(self, msg):
        filtered_msg = msg
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            if self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max:
                filtered_msg.ranges[i] = float('nan')  # Remove points inside box
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BoxFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
