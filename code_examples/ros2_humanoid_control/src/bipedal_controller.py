#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class BipedalController(Node):
    def __init__(self):
        super().__init__('bipedal_controller')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info('Bipedal Controller Node Started.')

    def timer_callback(self):
        # Placeholder for bipedal walking logic
        # In a real scenario, this would compute joint angles for walking
        joint_commands = Float32MultiArray()
        joint_commands.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Example: all joints to 0
        self.publisher_.publish(joint_commands)
        self.get_logger().info(f'Publishing joint commands: {joint_commands.data}')

def main(args=None):
    rclpy.init(args=args)
    bipedal_controller = BipedalController()
    rclpy.spin(bipedal_controller)
    bipedal_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
