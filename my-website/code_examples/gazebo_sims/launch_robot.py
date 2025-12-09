#!/usr/bin/env python3

import os
import rclpy
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

def main():
    rclpy.init()
    node = rclpy.create_node("spawn_humanoid")

    # Get URDF file path
    urdf_path = os.path.join(
        get_package_share_directory('humanoid_robot_description'), # Replace with your package name
        'urdf',
        'humanoid_robot.urdf'
    )

    # For now, let's just print a message and exit
    node.get_logger().info(f"Placeholder: Would spawn humanoid from {urdf_path} into Gazebo.")
    node.get_logger().info("Please run a Gazebo simulation separately.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
