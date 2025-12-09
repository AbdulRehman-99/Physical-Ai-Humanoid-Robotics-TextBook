#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AmbiguityHandler(Node):
    def __init__(self):
        super().__init__('ambiguity_handler')
        self.command_subscription = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        self.response_publisher = self.create_publisher(String, 'vla_response', 10)
        self.get_logger().info("Ambiguity Handler Node Started. Waiting for commands...")

        self.ambiguous_phrases = ["pick up the block", "move that", "get the thing"]

    def command_callback(self, msg):
        command = msg.data.lower()
        if any(phrase in command for phrase in self.ambiguous_phrases):
            response = "I'm sorry, I need more clarification. Which object are you referring to?"
            self.get_logger().warn(f"Ambiguous command received: '{command}'. Responding: '{response}'")
            self.response_publisher.publish(String(data=response))
        else:
            self.get_logger().info(f"Command '{command}' seems clear. Passing to VLA system.")
            # In a real system, forward to the VLA processing node

def main(args=None):
    rclpy.init(args=args)
    ambiguity_handler = AmbiguityHandler()
    rclpy.spin(ambiguity_handler)
    ambiguity_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
