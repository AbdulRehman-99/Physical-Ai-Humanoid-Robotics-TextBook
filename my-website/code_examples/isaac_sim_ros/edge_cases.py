#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class NoisySensorHandler(Node):
    def __init__(self):
        super().__init__('noisy_sensor_handler')
        self.subscription = self.create_subscription(
            LaserScan,
            'noisy_scan', # Assuming a noisy scan topic
            self.noisy_scan_callback,
            10
        )
        self.publisher_ = self.create_publisher(LaserScan, 'filtered_scan', 10)
        self.get_logger().info("Noisy Sensor Handler Node Started. Waiting for noisy scans...")

    def noisy_scan_callback(self, msg):
        # Placeholder for sensor data filtering logic
        # In a real scenario, this would apply filters (e.g., median filter, Kalman filter)
        # to smooth out the noisy data.
        self.get_logger().info("Received noisy scan data. Applying filtering (placeholder)...")

        filtered_ranges = list(msg.ranges)
        # Simulate some filtering: e.g., remove extreme outliers
        for i in range(len(filtered_ranges)):
            if filtered_ranges[i] > msg.range_max * 0.9: # Example: too far, likely noise
                filtered_ranges[i] = float('inf')
            elif filtered_ranges[i] < msg.range_min * 1.1: # Example: too close, likely noise
                filtered_ranges[i] = float('inf')
            # Add more sophisticated filtering here

        filtered_msg = msg
        filtered_msg.ranges = filtered_ranges
        self.publisher_.publish(filtered_msg)
        self.get_logger().info("Published filtered scan data.")


def main(args=None):
    rclpy.init(args=args)
    noisy_sensor_handler = NoisySensorHandler()
    rclpy.spin(noisy_sensor_handler)
    noisy_sensor_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
