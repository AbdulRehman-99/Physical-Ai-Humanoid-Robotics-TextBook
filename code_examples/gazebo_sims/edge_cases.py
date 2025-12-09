#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

class CommandValidator(rclpy.node.Node):
    def __init__(self):
        super().__init__('command_validator')
        self.subscription = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Command Validator Node Started. Waiting for commands...")

    def command_callback(self, msg):
        command = msg.data.lower()
        if command in ['wave', 'stand', 'sit', 'walk']:
            self.get_logger().info(f"Received valid command: {command}")
            # In a real scenario, you would forward this to the robot controller
        else:
            self.get_logger().warn(f"Received invalid command: {command}. Ignoring or handling gracefully.")
            # Log the invalid command, send error feedback, etc.

def main(args=None):
    rclpy.init(args=args)
    command_validator = CommandValidator()
    rclpy.spin(command_validator)
    command_validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
