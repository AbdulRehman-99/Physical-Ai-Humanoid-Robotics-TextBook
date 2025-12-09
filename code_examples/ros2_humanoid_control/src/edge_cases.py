#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class GaitStabilizer(Node):
    def __init__(self):
        super().__init__('gait_stabilizer')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_commands',
            self.command_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'stable_joint_commands', 10)
        self.get_logger().info("Gait Stabilizer Node Started. Waiting for joint commands...")

    def command_callback(self, msg):
        # Placeholder for gait stabilization logic
        # In a real scenario, this would analyze joint commands for stability
        # and potentially adjust them to prevent falls.
        unstable_gait = False # Example condition for unstable gait

        # Simulate detecting an unstable gait parameter
        if any(abs(angle) > 1.5 for angle in msg.data): # Example: large joint angles
            unstable_gait = True

        if unstable_gait:
            self.get_logger().warn("Detected potentially unstable gait parameters. Adjusting commands.")
            # Example: Send a "stand still" command
            stable_commands = Float32MultiArray()
            stable_commands.data = [0.0] * len(msg.data)
            self.publisher_.publish(stable_commands)
        else:
            self.get_logger().info(f"Received stable joint commands: {msg.data}")
            self.publisher_.publish(msg) # Pass through if stable

def main(args=None):
    rclpy.init(args=args)
    gait_stabilizer = GaitStabilizer()
    rclpy.spin(gait_stabilizer)
    gait_stabilizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
