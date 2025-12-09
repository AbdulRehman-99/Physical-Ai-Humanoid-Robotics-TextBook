#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('simple_command_publisher')

    publisher = node.create_publisher(String, 'robot_command', 10)

    msg = String()
    msg.data = 'wave'  # Example command

    node.get_logger().info(f'Publishing: "{msg.data}" to robot_command topic')
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
