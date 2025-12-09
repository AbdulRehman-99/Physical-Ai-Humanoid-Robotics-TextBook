from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_humanoid_control', # Replace with your package name
            executable='bipedal_controller.py',
            name='bipedal_controller',
            output='screen'
        ),
        Node(
            package='ros2_humanoid_control', # Replace with your package name
            executable='gait_stabilizer.py',
            name='gait_stabilizer',
            output='screen'
        ),
        # Add Gazebo launch here if you want to launch it from this file
        # For now, assume Gazebo is launched separately
    ])
