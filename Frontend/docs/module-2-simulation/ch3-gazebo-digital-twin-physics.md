---
title: "Chapter 3: Gazebo - Digital Twin & Physics"
sidebar_position: 1
---

# Chapter 3: Gazebo - Digital Twin & Physics

## Introduction to Gazebo and Digital Twin Technology

Gazebo stands as one of the most prominent physics simulation environments in robotics, serving as a critical component in the development and testing of humanoid robots. As a digital twin technology, Gazebo creates a virtual replica of the physical robot and its environment, enabling developers to test algorithms, validate control strategies, and debug systems without the risks and costs associated with physical hardware. This capability is particularly valuable for humanoid robotics, where the complexity of the systems and the potential for damage to expensive hardware makes simulation an essential tool.

The concept of a digital twin extends beyond simple visualization to encompass accurate physical modeling, sensor simulation, and environmental interaction. In the context of humanoid robotics, a digital twin must faithfully reproduce the robot's kinematics, dynamics, sensor characteristics, and the physical properties of the environment in which it operates. This requires sophisticated modeling of:

- **Multi-body dynamics**: Accurate simulation of joint interactions, collisions, and force propagation
- **Sensor modeling**: Realistic simulation of cameras, lidar, IMUs, and other sensors
- **Environmental physics**: Accurate representation of friction, gravity, and material properties
- **Control system integration**: Seamless connection between simulation and control algorithms

Gazebo's architecture is built around the Open Dynamics Engine (ODE), Bullet Physics, or DART physics engines, which provide the mathematical foundation for simulating rigid body dynamics. These engines handle complex calculations involving mass, inertia, friction, and collision detection, allowing for realistic simulation of humanoid robot behavior including walking, balance, and manipulation tasks.

## Physics Simulation Fundamentals

### Rigid Body Dynamics

The foundation of Gazebo's physics simulation lies in rigid body dynamics, which models objects as collections of mass points connected by constraints. For humanoid robots, this means each link (arm, leg, torso) is treated as a rigid body with specific mass, center of mass, and moment of inertia properties. The physics engine calculates how forces and torques applied to these bodies result in motion, taking into account:

- **Newton's laws of motion**: Governing the relationship between forces and motion
- **Euler's rotation equations**: Describing rotational motion of rigid bodies
- **Constraint equations**: Modeling joints and their allowed degrees of freedom
- **Collision detection and response**: Handling interactions between bodies

The accuracy of rigid body dynamics is crucial for humanoid robotics because balance and locomotion depend on precise understanding of how forces propagate through the robot's structure. Small errors in simulation can lead to significant differences in behavior between simulated and real robots, making accurate modeling essential.

### Collision Detection and Response

Collision detection in Gazebo involves two main phases: broad phase and narrow phase collision detection. The broad phase uses spatial partitioning to quickly identify potentially colliding pairs of objects, while the narrow phase performs detailed geometric calculations to determine if and where collisions actually occur.

For humanoid robots, collision detection is particularly important for:
- **Self-collision avoidance**: Preventing limbs from intersecting with each other
- **Environment interaction**: Modeling contact with floors, walls, and objects
- **Balance control**: Accurate ground contact modeling for walking and standing
- **Manipulation**: Precise contact modeling for grasping and object interaction

Gazebo supports various collision geometries including boxes, spheres, cylinders, and meshes, allowing for accurate representation of complex humanoid robot shapes. The collision response model determines how objects react when they contact each other, including factors like friction, restitution (bounciness), and surface properties.

### Joint Modeling and Constraints

Humanoid robots require sophisticated joint modeling to accurately simulate their range of motion and mechanical properties. Gazebo supports several joint types that are essential for humanoid simulation:

- **Revolute joints**: Single-axis rotation with position, velocity, and effort limits
- **Prismatic joints**: Single-axis translation
- **Fixed joints**: Rigid connections between bodies
- **Continuous joints**: Unconstrained single-axis rotation
- **Floating joints**: 6 degrees of freedom
- **Planar joints**: Movement in a plane

Each joint type can be configured with various parameters including limits, damping, friction, and actuator properties. For humanoid robots, these parameters must be carefully tuned to match the physical robot's characteristics, including gear ratios, motor limits, and mechanical compliance.

## Gazebo World Design and Environment Modeling

### World Structure and XML Format

Gazebo worlds are defined using SDF (Simulation Description Format), an XML-based format that describes the complete simulation environment. A typical Gazebo world file includes:

- **World properties**: Gravity, magnetic field, atmosphere settings
- **Models**: Robot models, objects, and static structures
- **Lights**: Point lights, directional lights, spotlights
- **Plugins**: Additional functionality for sensors, controllers, or custom behavior
- **Physics engine settings**: Solver parameters, step size, real-time update rate

The world file serves as the foundation for simulation, defining the environment in which the humanoid robot operates. For humanoid robotics applications, the world typically includes:

- **Ground planes**: With appropriate friction and restitution properties
- **Obstacles**: To test navigation and obstacle avoidance
- **Interactive objects**: For manipulation and task performance testing
- **Sensors and actuators**: To provide environmental feedback

### Creating Realistic Environments

Realistic environment modeling is crucial for humanoid robotics simulation, as the robot must interact with the environment in ways that closely match real-world behavior. Key considerations include:

**Surface Properties**: The friction and restitution coefficients of surfaces significantly impact humanoid robot behavior, particularly for walking and balance. High-friction surfaces provide better grip for walking, while low-friction surfaces may cause slipping and require different control strategies.

**Terrain Modeling**: Humanoid robots must navigate various terrain types, from flat floors to stairs, ramps, and uneven surfaces. Gazebo can model these terrains using height maps, mesh models, or primitive shapes with appropriate collision properties.

**Dynamic Objects**: Environments often include objects that can be moved or manipulated by the robot. These objects require accurate mass properties, collision geometry, and may need to be configured with specific joint constraints or degrees of freedom.

**Lighting and Visibility**: For robots with vision systems, proper lighting is essential for realistic sensor simulation. Gazebo supports various lighting models and can simulate shadows, reflections, and atmospheric effects.

### Advanced World Features

Gazebo includes several advanced features that enhance the realism and utility of humanoid robot simulations:

**Wind Simulation**: Can be used to test robot stability under environmental forces, important for balance control algorithms.

**Multi-robot Simulation**: Allows for testing of multi-robot scenarios, human-robot interaction, and collaborative behaviors.

**Plugin Architecture**: Enables extension of simulation capabilities with custom sensors, controllers, or environmental effects.

**ROS Integration**: Seamless integration with ROS/ROS 2 for realistic sensor simulation and control system testing.

## Sensor Simulation in Gazebo

### Camera and Vision Sensors

Vision sensors in Gazebo provide realistic simulation of cameras used on humanoid robots for perception, navigation, and interaction. The camera simulation includes:

- **Intrinsic parameters**: Focal length, principal point, distortion coefficients
- **Extrinsic parameters**: Position and orientation relative to the robot
- **Image properties**: Resolution, frame rate, noise characteristics
- **Distortion modeling**: Realistic simulation of lens distortion effects

For humanoid robots, vision sensors are typically mounted on the head for environmental perception and on hands for manipulation tasks. The simulation must accurately model the field of view, resolution, and noise characteristics to provide realistic perception challenges.

### IMU and Inertial Sensors

Inertial Measurement Units (IMUs) are critical for humanoid robot balance and motion control. Gazebo's IMU simulation includes:

- **Accelerometer modeling**: Linear acceleration measurement with noise and bias
- **Gyroscope modeling**: Angular velocity measurement with drift and noise
- **Magnetometer modeling**: Magnetic field measurement for heading reference
- **Frame alignment**: Proper mounting orientation relative to robot body

IMU data is essential for humanoid balance control, providing information about the robot's orientation, acceleration, and angular velocity. The simulation must accurately model sensor noise, bias, and drift to provide realistic challenges for control algorithms.

### Force/Torque Sensors

Force and torque sensors are important for humanoid robot manipulation and balance. Gazebo can simulate these sensors at various points on the robot, including:

- **Joint force/torque sensors**: Measuring forces and torques at individual joints
- **Force-torque sensors**: Typically mounted at wrists for manipulation tasks
- **Pressure sensors**: Simulating foot pressure distribution for balance control

These sensors provide critical feedback for control algorithms, particularly for tasks requiring precise force control or balance maintenance.

### Lidar and Range Sensors

Lidar and range sensors provide distance measurements used for navigation, mapping, and obstacle detection. Gazebo's range sensor simulation includes:

- **Scan pattern**: Angular resolution and field of view
- **Range limits**: Minimum and maximum measurable distances
- **Accuracy modeling**: Measurement noise and uncertainty
- **Ray tracing**: Accurate modeling of beam propagation and reflection

For humanoid robots, range sensors are typically mounted on the head or torso for environment mapping and obstacle detection during navigation.

## Integration with ROS and Control Systems

### Gazebo-ROS Bridge

The integration between Gazebo and ROS/ROS 2 is facilitated by the `gazebo_ros` package, which provides plugins and interfaces for seamless communication between the simulation and control systems. This integration enables:

- **Sensor data publishing**: Real-time publishing of simulated sensor data to ROS topics
- **Actuator control**: Receiving control commands from ROS and applying them to simulated joints
- **TF broadcasting**: Publishing coordinate transforms for spatial reasoning
- **Model state management**: Controlling robot and object poses in simulation

The Gazebo-ROS bridge is essential for humanoid robotics development, allowing the same control algorithms to run in simulation and on real hardware with minimal modification.

### Control System Integration

Humanoid robot control systems in Gazebo typically use the ROS 2 Control framework, which provides a standardized interface for hardware abstraction. This framework includes:

- **Hardware interfaces**: Standardized interfaces for joint position, velocity, and effort control
- **Controllers**: Pre-built controllers for common tasks like joint trajectory following
- **Resource managers**: Managing access to hardware resources and preventing conflicts
- **Real-time capabilities**: Support for deterministic control loops

For humanoid robots, the control system integration must handle the complexity of multi-joint systems with coordinated motion, balance requirements, and safety considerations.

### Simulation Speed and Real-time Factors

Gazebo simulation can run at different speeds relative to real time, controlled by the real-time update rate and max step size parameters:

- **Real-time factor**: The ratio of simulation time to real time (1.0 = real-time, >1.0 = faster than real-time)
- **Update rate**: How frequently the physics engine updates (typically 1000 Hz for accurate simulation)
- **Step size**: The time increment for each physics calculation (typically 0.001 seconds)

For humanoid robotics, the simulation parameters must balance accuracy and computational efficiency. Too large a step size can cause instability in control systems, while too small a step size can create excessive computational load.

## Advanced Gazebo Features for Humanoid Robotics

### Contact and Force Analysis

Gazebo provides detailed information about contacts and forces between objects, which is crucial for humanoid robotics applications:

- **Contact detection**: Detailed information about where and when contacts occur
- **Force calculation**: Accurate computation of contact forces and torques
- **Pressure distribution**: Modeling of pressure across contact surfaces
- **Friction modeling**: Advanced friction models including static and dynamic friction

This information is essential for humanoid balance control, where understanding ground reaction forces and pressure distribution is critical for maintaining stability.

### Plugin Development

Gazebo's plugin architecture allows for custom functionality tailored to specific humanoid robotics applications:

- **Model plugins**: Custom behavior for specific robot models
- **World plugins**: Custom world behavior and simulation features
- **Sensor plugins**: Custom sensor models or processing
- **System plugins**: Low-level system modifications

Plugins can implement complex behaviors like custom control algorithms, environmental effects, or specialized sensor models that are not available in the standard Gazebo distribution.

### Multi-Physics Simulation

Advanced Gazebo configurations can include additional physics modeling beyond rigid body dynamics:

- **Fluid simulation**: Modeling of liquid environments or fluid-structure interaction
- **Soft body physics**: Modeling of deformable objects
- **Particle systems**: Simulation of dust, smoke, or other particle effects
- **Thermal modeling**: Temperature effects on robot performance

While not always necessary for basic humanoid robotics, these features can be valuable for specific applications or research scenarios.

## Best Practices for Humanoid Robot Simulation

### Model Accuracy and Validation

Ensuring that the simulated robot accurately represents the physical robot is crucial for effective development:

- **Mass properties**: Accurate mass, center of mass, and moment of inertia for each link
- **Joint limits**: Correct position, velocity, and effort limits matching the physical robot
- **Sensor noise**: Realistic noise models based on physical sensor characteristics
- **Actuator dynamics**: Accurate modeling of motor response, gear ratios, and limitations

Regular validation against physical robot behavior helps maintain simulation accuracy and identifies areas where the model may need refinement.

### Performance Optimization

Large-scale humanoid robotics simulations can be computationally intensive, requiring optimization strategies:

- **Simplification**: Using simplified collision geometries where accuracy allows
- **Level of detail**: Adjusting simulation detail based on task requirements
- **Parallel processing**: Utilizing multi-core processors for physics calculations
- **Resource management**: Efficient use of memory and computational resources

### Debugging and Visualization

Gazebo provides extensive debugging and visualization tools for humanoid robot development:

- **Physics visualization**: Display of contact forces, collision geometries, and joint axes
- **Sensor visualization**: Real-time display of sensor data and field of view
- **Trajectory visualization**: Display of planned and executed motion paths
- **Logging and analysis**: Detailed logging for post-simulation analysis

These tools are invaluable for understanding robot behavior, identifying problems, and validating control algorithms.

## Conclusion

Gazebo provides the essential simulation environment for humanoid robotics development, offering realistic physics modeling, sensor simulation, and integration with control systems. The ability to create accurate digital twins of humanoid robots enables safe, efficient development and testing of complex behaviors before deployment on physical hardware.

The integration with ROS/ROS 2 provides a seamless development pipeline where algorithms can be tested in simulation and deployed on real robots with minimal modification. As humanoid robotics continues to advance, simulation environments like Gazebo will remain critical tools for developing and validating the sophisticated control systems required for these complex machines.

The next chapter will explore Unity as an alternative simulation platform, focusing on high-fidelity graphics and human-robot interaction capabilities that complement the physics-focused approach of Gazebo.

---

## References

[1] Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2350-2354.

[2] Gazebo Sim. (2023). Gazebo Documentation. Retrieved from https://gazebosim.org/

[3] ROS.org. (2023). Gazebo Tutorials. Retrieved from http://gazebosim.org/tutorials

[4] Tedrake, R. (2023). Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation. MIT Press.

[5] Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics*. Springer.

## Code Examples

### Example 1: Basic Gazebo World File with Humanoid Robot
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Humanoid robot model -->
    <model name="humanoid_robot">
      <pose>0 0 1 0 0 0</pose>
      <include>
        <uri>model://humanoid_description</uri>
      </include>
    </model>

    <!-- Simple environment objects -->
    <model name="table">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="table_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add plugins for ROS integration -->
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
    </plugin>
  </world>
</sdf>
```

### Example 2: Gazebo Model Configuration for Humanoid Robot
```xml
<?xml version="1.0" ?>
<robot name="humanoid_model" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include other xacro files if needed -->
  <xacro:include filename="$(find-pkg-share humanoid_description)/urdf/materials.xacro" />
  <xacro:include filename="$(find-pkg-share humanoid_description)/urdf/properties.xacro" />

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
      <material name="light_grey" />
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.2 0.2 0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Torso link -->
  <link name="torso">
    <inertial>
      <mass value="8.0" />
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.2 0.6" />
      </geometry>
      <material name="blue" />
    </visual>

    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <geometry>
        <box size="0.3 0.2 0.6" />
      </geometry>
    </collision>
  </link>

  <!-- Joint between base and torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
  </joint>

  <!-- Left leg chain -->
  <joint name="left_hip_pitch" type="revolute">
    <parent link="torso" />
    <child link="left_thigh" />
    <origin xyz="0 -0.1 -0.1" rpy="0 0 0" />
    <axis xyz="1 0 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2" />
    <dynamics damping="1.0" friction="0.1" />
  </joint>

  <link name="left_thigh">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01" />
    </inertial>

    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.4" />
      </geometry>
      <material name="red" />
    </visual>

    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.4" />
      </geometry>
    </collision>
  </link>

  <!-- Additional joints and links for complete humanoid model -->
  <!-- ... (additional joints and links) ... -->

  <!-- Gazebo-specific tags -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_thigh">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- Transmission for ROS control -->
  <transmission name="left_hip_pitch_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_pitch_motor">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Sensor definitions -->
  <gazebo reference="head">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head_camera_optical_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <frame_name>imu_link</frame_name>
        <body_name>torso</body_name>
        <topic>__default_topic__</topic>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Example 3: ROS 2 Node for Controlling Humanoid in Gazebo
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class GazeboHumanoidController(Node):
    def __init__(self):
        super().__init__('gazebo_humanoid_controller')

        # Publishers for different control interfaces
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribers for sensor feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        # Internal state
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.imu_orientation = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = None

        self.get_logger().info('Gazebo Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Update internal joint state from sensor feedback"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update internal IMU state from sensor feedback"""
        self.imu_orientation = msg.orientation
        self.imu_angular_velocity = msg.angular_velocity
        self.imu_linear_acceleration = msg.linear_acceleration

    def control_loop(self):
        """Main control loop for humanoid robot in simulation"""
        # Example: Simple balance controller using IMU feedback
        if self.imu_orientation is not None:
            # Extract roll and pitch from quaternion
            quat = self.imu_orientation
            roll, pitch, yaw = self.quaternion_to_euler(
                quat.x, quat.y, quat.z, quat.w
            )

            # Simple balance control based on pitch angle
            target_joint_positions = self.balance_control(pitch, roll)

            # Send trajectory command
            self.send_trajectory_command(target_joint_positions)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def balance_control(self, pitch, roll):
        """Simple balance control based on IMU feedback"""
        # Calculate desired joint adjustments based on orientation error
        pitch_correction = -pitch * 10.0  # Proportional control
        roll_correction = -roll * 10.0

        # Return target positions for key joints
        # This is a simplified example - real balance control is much more complex
        return {
            'left_hip_pitch': pitch_correction,
            'right_hip_pitch': pitch_correction,
            'left_ankle_pitch': -pitch_correction,
            'right_ankle_pitch': -pitch_correction,
            'left_hip_roll': roll_correction,
            'right_hip_roll': roll_correction,
            'left_ankle_roll': -roll_correction,
            'right_ankle_roll': -roll_correction
        }

    def send_trajectory_command(self, joint_positions):
        """Send trajectory command to robot joints"""
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_positions.keys())

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions.values())
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

        trajectory.points = [point]

        self.joint_trajectory_pub.publish(trajectory)

    def send_walk_command(self, step_length=0.1, step_height=0.05):
        """Send a simple walking command"""
        # This would implement a more complex walking pattern
        # For demonstration, just send a simple stepping motion
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip_pitch', 'left_knee', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee', 'right_ankle_pitch'
        ]

        # Define a simple step trajectory
        points = []
        for i in range(10):
            point = JointTrajectoryPoint()
            # Generate trajectory points for stepping motion
            t = i / 10.0
            left_hip = step_length * math.sin(math.pi * t)
            right_hip = -step_length * math.sin(math.pi * t)
            left_knee = step_height * math.sin(math.pi * t)
            right_knee = step_height * math.sin(math.pi * t)

            point.positions = [
                left_hip, left_knee, 0.0,
                right_hip, right_knee, 0.0
            ]
            point.time_from_start = Duration(
                sec=int(t * 2),
                nanosec=int((t * 2 - int(t * 2)) * 1e9)
            )
            points.append(point)

        trajectory.points = points
        self.joint_trajectory_pub.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboHumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 4: Gazebo Launch File for Humanoid Simulation
```xml
<launch>
  <!-- Arguments -->
  <arg name="world" default="humanoid_world"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="verbose" default="false"/>

  <!-- Set environment variables -->
  <env name="GAZEBO_MODEL_PATH" value="$(find-pkg-share humanoid_description)/models:$(find-pkg-share gazebo_models)"/>
  <env name="GAZEBO_RESOURCE_PATH" value="$(find-pkg-share gazebo_resources)"/>

  <!-- Launch Gazebo -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gzserver.launch.py">
    <arg name="world" value="$(find-pkg-share humanoid_gazebo)/worlds/$(var world).world"/>
    <arg name="verbose" value="$(var verbose)"/>
    <arg name="pause" value="false"/>
  </include>

  <!-- Launch Gazebo client if GUI is enabled -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gzclient.launch.py" if="$(var gui)">
  </include>

  <!-- Spawn robot in Gazebo -->
  <node pkg="gazebo_ros" exec="spawn_entity.py" output="screen"
        args="-entity humanoid_robot -topic robot_description -x 0 -y 0 -z 1.0"/>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
    <param name="rate" value="50"/>
  </node>

  <!-- Load controller configurations -->
  <node pkg="controller_manager" exec="ros2_control_node" name="ros2_control_node">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Launch controller spawners -->
  <node pkg="controller_manager" exec="spawner" args="joint_state_controller"/>
  <node pkg="controller_manager" exec="spawner" args="joint_trajectory_controller"/>

  <!-- Launch humanoid controller -->
  <node pkg="humanoid_control" exec="gazebo_humanoid_controller" name="gazebo_humanoid_controller" output="screen">
    <param name="control_frequency" value="100"/>
    <param name="balance_control_enabled" value="true"/>
  </node>
</launch>
```

### Example 5: Gazebo Plugin for Custom Sensor Simulation
```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace gazebo
{
  class HumanoidSensorPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->world = _model->GetWorld();

      // Get parameters from SDF
      if (_sdf->HasElement("robot_namespace"))
        this->robotNamespace = _sdf->Get<std::string>("robot_namespace");
      else
        this->robotNamespace = "/humanoid_robot";

      // Initialize ROS
      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

      this->ros_node = rclcpp::Node::make_shared("gazebo_humanoid_sensor_plugin");

      // Create publishers
      this->contact_publisher = this->ros_node->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        this->robotNamespace + "/ground_contact", 10);

      // Connect to world update event
      this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HumanoidSensorPlugin::OnUpdate, this));

      RCLCPP_INFO(this->ros_node->get_logger(), "Humanoid Sensor Plugin loaded");
    }

    public: void OnUpdate()
    {
      // Get current time
      auto current_time = this->world->SimTime();

      // Check for ground contact on feet
      auto left_foot_link = this->model->GetLink("left_foot");
      auto right_foot_link = this->model->GetLink("right_foot");

      if (left_foot_link && right_foot_link)
      {
        // Get contact information
        auto contacts = this->world->Physics()->GetContacts();

        // Publish contact information for left foot
        geometry_msgs::msg::Vector3Stamped left_contact_msg;
        left_contact_msg.header.stamp.sec = current_time.sec;
        left_contact_msg.header.stamp.nanosec = current_time.nsec;
        left_contact_msg.header.frame_id = "left_foot";

        // Calculate contact forces (simplified)
        left_contact_msg.vector.x = 0.0;
        left_contact_msg.vector.y = 0.0;
        left_contact_msg.vector.z = this->calculateGroundForce(left_foot_link);

        this->contact_publisher->publish(left_contact_msg);

        // Publish contact information for right foot
        geometry_msgs::msg::Vector3Stamped right_contact_msg;
        right_contact_msg.header.stamp.sec = current_time.sec;
        right_contact_msg.header.stamp.nanosec = current_time.nsec;
        right_contact_msg.header.frame_id = "right_foot";

        right_contact_msg.vector.x = 0.0;
        right_contact_msg.vector.y = 0.0;
        right_contact_msg.vector.z = this->calculateGroundForce(right_foot_link);

        this->contact_publisher->publish(right_contact_msg);
      }

      // Spin ROS node
      rclcpp::spin_some(this->ros_node);
    }

    private: double calculateGroundForce(physics::LinkPtr link)
    {
      // Simplified ground force calculation
      // In a real implementation, this would use contact information
      auto linear_vel = link->WorldLinearVel();
      auto position = link->WorldPose().Pos();

      // Return a value based on height above ground (simplified)
      return std::max(0.0, 100.0 * (0.05 - position.Z())); // 5cm above ground threshold
    }

    private: physics::ModelPtr model;
    private: physics::WorldPtr world;
    private: std::string robotNamespace;
    private: rclcpp::Node::SharedPtr ros_node;
    private: rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr contact_publisher;
    private: event::ConnectionPtr update_connection;
  };

  GZ_REGISTER_MODEL_PLUGIN(HumanoidSensorPlugin)
}
```