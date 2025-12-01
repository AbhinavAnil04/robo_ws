#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # --- Original APTELIDAR node ---
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # --- Box Filter node ---
        Node(
            package='lidar_box_filter',
            executable='box_filter',
            output='screen',
            parameters=[{
                'x_min': -0.28,
                'x_max': 0.28,
                'y_min': -0.28,
                'y_max': 0.28
            }]
        )
    ])
