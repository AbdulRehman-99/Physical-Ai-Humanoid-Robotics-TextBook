---
title: "Chapter 6: Isaac ROS - VSLAM, Localization & Nav2 for Humanoids"
sidebar_position: 2
---

# Chapter 6: Isaac ROS - VSLAM, Localization & Nav2 for Humanoids

## Introduction to Isaac ROS and Visual SLAM

Isaac ROS represents NVIDIA's specialized implementation of ROS 2 nodes optimized for perception and navigation tasks using NVIDIA's hardware acceleration. Built specifically for robotics applications requiring high-performance perception and navigation, Isaac ROS provides GPU-accelerated implementations of critical algorithms including Visual Simultaneous Localization and Mapping (VSLAM), localization, and navigation systems. For humanoid robots, Isaac ROS offers the computational power and specialized algorithms necessary to handle the complex perception and navigation challenges these systems face in dynamic environments.

Isaac ROS extends traditional ROS 2 by providing hardware-accelerated nodes that leverage NVIDIA's GPUs and specialized processing units like Tensor Cores and RT Cores. This acceleration is crucial for humanoid robotics where real-time performance is essential for safety and responsiveness. The framework includes optimized implementations of computer vision algorithms, sensor processing, and navigation systems that can handle the high data rates and computational requirements of humanoid robot perception systems.

The integration of Isaac ROS with traditional ROS 2 navigation systems, particularly Nav2 (Navigation 2), creates a powerful platform for developing sophisticated navigation capabilities for humanoid robots. This combination allows for the development of systems that can perceive, map, localize, and navigate in complex environments while maintaining the real-time performance requirements essential for humanoid robot operation.

### Isaac ROS Architecture and Components

Isaac ROS is built around a modular architecture that allows for flexible integration with existing ROS 2 systems:

**Hardware Acceleration Layer**: Leverages NVIDIA GPUs and specialized hardware:
- **CUDA Integration**: Direct GPU computing for algorithm acceleration
- **TensorRT Integration**: Optimized neural network inference
- **Vision Accelerators**: Hardware-accelerated computer vision operations
- **Deep Learning Accelerators**: Specialized processing for AI algorithms

**Perception Pipeline**: Optimized processing for sensor data:
- **Camera Processing**: High-performance image processing and analysis
- **LiDAR Processing**: Accelerated point cloud processing
- **Sensor Fusion**: Real-time combination of multiple sensor modalities
- **Feature Detection**: GPU-accelerated feature extraction and matching

**Navigation Components**: Specialized navigation algorithms:
- **SLAM Systems**: GPU-accelerated simultaneous localization and mapping
- **Path Planning**: Optimized pathfinding for complex environments
- **Trajectory Generation**: Real-time trajectory planning for humanoid motion
- **Obstacle Avoidance**: Accelerated collision detection and avoidance

### Isaac ROS in the Robotics Ecosystem

Isaac ROS fills a critical gap in the robotics ecosystem by providing high-performance implementations of perception and navigation algorithms:

- **Performance Enhancement**: GPU acceleration for computationally intensive tasks
- **Real-time Processing**: Low-latency processing for safety-critical applications
- **AI Integration**: Seamless integration with NVIDIA's AI frameworks
- **ROS 2 Compatibility**: Full compatibility with existing ROS 2 systems
- **Hardware Optimization**: Tailored for NVIDIA hardware platforms

## Visual SLAM (VSLAM) Fundamentals

### SLAM Theory and Principles

Simultaneous Localization and Mapping (SLAM) is a fundamental problem in robotics where a robot must build a map of an unknown environment while simultaneously localizing itself within that map. Visual SLAM (VSLAM) specifically uses visual sensors (cameras) as the primary input for this process. For humanoid robots, VSLAM is particularly important because it enables the robot to understand its environment using the same visual information that humans use for navigation and spatial awareness.

The VSLAM problem involves several key components:

**State Estimation**: Estimating the robot's pose (position and orientation) in the environment over time. This is typically formulated as a probabilistic estimation problem where the robot maintains a belief about its pose based on sensor measurements and motion models.

**Map Building**: Creating and maintaining a representation of the environment that can be used for navigation and localization. The map representation can take various forms including point clouds, occupancy grids, or semantic maps.

**Data Association**: Determining which sensor measurements correspond to which features in the map. This is crucial for updating the map and refining the pose estimate.

**Loop Closure**: Detecting when the robot returns to a previously visited location to correct accumulated errors in the map and pose estimate.

### VSLAM Approaches and Algorithms

Several approaches to VSLAM exist, each with different trade-offs in terms of accuracy, computational requirements, and robustness:

**Feature-Based VSLAM**: Extracts and tracks distinctive visual features across frames:
- **ORB-SLAM**: Uses ORB (Oriented FAST and Rotated BRIEF) features
- **LSD-SLAM**: Direct method that works with image intensities
- **SVO**: Semi-direct visual odometry approach

**Direct VSLAM**: Works directly with image intensities rather than extracted features:
- **LSD-SLAM**: Large-Scale Direct SLAM
- **DSO**: Direct Sparse Odometry
- **REMODE**: Real-time stereo reconstruction

**Semantic VSLAM**: Incorporates semantic information to improve mapping and localization:
- **Object-based SLAM**: Uses object detection and tracking
- **Semantic mapping**: Incorporates semantic labels into the map

### Isaac ROS VSLAM Implementation

Isaac ROS provides optimized implementations of VSLAM algorithms that leverage NVIDIA's hardware acceleration:

**Hardware Acceleration**: VSLAM algorithms in Isaac ROS are optimized for GPU execution:
- **CUDA-based Feature Detection**: Accelerated feature extraction and matching
- **Parallel Processing**: Multi-threaded and GPU-parallelized algorithms
- **Memory Optimization**: Efficient memory management for large datasets
- **Real-time Performance**: Optimized for real-time operation on humanoid robots

**Sensor Integration**: Isaac ROS VSLAM systems can integrate multiple sensor modalities:
- **Stereo Cameras**: Depth estimation from stereo vision
- **RGB-D Cameras**: Direct depth information integration
- **Multi-camera Systems**: Integration of multiple camera viewpoints
- **IMU Integration**: Inertial measurement unit data fusion

## Localization Systems for Humanoid Robots

### Pose Estimation and Tracking

Localization in robotics refers to the process of determining the robot's position and orientation (pose) in a known or unknown environment. For humanoid robots, accurate localization is critical for navigation, interaction, and safety. The localization system must handle the unique challenges of humanoid robot motion including bipedal locomotion, which introduces complex dynamics and potential for drift in pose estimation.

**Absolute Localization**: Determining the robot's pose in a global coordinate system:
- **GPS Integration**: For outdoor humanoid robots
- **Beacon Systems**: Using known landmarks or beacons
- **Visual Landmarks**: Using distinctive visual features
- **RFID/NFC Tags**: Using embedded location markers

**Relative Localization**: Tracking pose changes relative to a starting position:
- **Visual Odometry**: Tracking motion using visual features
- **Inertial Navigation**: Using IMU data for motion tracking
- **Wheel Encoders**: Tracking motion using joint encoders
- **LiDAR Odometry**: Tracking motion using LiDAR data

### Multi-Sensor Fusion for Localization

Humanoid robots typically use multiple sensors for robust localization, requiring sophisticated fusion algorithms:

**Kalman Filtering**: Optimal estimation for linear systems with Gaussian noise:
- **Extended Kalman Filter (EKF)**: For non-linear systems
- **Unscented Kalman Filter (UKF)**: Better handling of non-linearities
- **Information Filter**: Dual representation of Kalman filtering

**Particle Filtering**: Non-parametric approach for non-Gaussian systems:
- **Monte Carlo Localization**: Grid-based particle filtering
- **Rao-Blackwellized Particle Filter**: Mixed discrete-continuous state spaces
- **Adaptive Particle Filtering**: Dynamic adjustment of particle count

**Factor Graph Optimization**: Maximum likelihood estimation using graph optimization:
- **g2o**: General framework for graph optimization
- **Ceres Solver**: General-purpose optimization framework
- **GTSAM**: Georgia Tech Smoothing and Mapping library

### Isaac ROS Localization Tools

Isaac ROS provides specialized tools for localization that leverage NVIDIA's hardware acceleration:

**GPU-Accelerated Feature Matching**: Fast and robust feature matching:
- **CUDA-based Descriptor Matching**: Accelerated descriptor comparison
- **Parallel Tracking**: Multiple feature tracking in parallel
- **Real-time Performance**: Maintaining high frame rates for humanoid control

**Optimized Filtering Algorithms**: GPU-accelerated state estimation:
- **Parallel Kalman Filtering**: Multiple Kalman filters in parallel
- **GPU-based Optimization**: Accelerated graph optimization
- **Real-time Constraints**: Maintaining real-time performance for safety

## Navigation Systems: Nav2 for Humanoid Robots

### Nav2 Architecture and Components

Navigation2 (Nav2) is the next-generation navigation framework for ROS 2, designed to provide robust, flexible, and extensible navigation capabilities. For humanoid robots, Nav2 provides the foundation for autonomous navigation in complex environments, handling the unique challenges of bipedal locomotion and human-scale environments.

Nav2 is built around a behavior tree architecture that allows for flexible composition of navigation behaviors:

**Global Planner**: Generates optimal paths from start to goal:
- **A* Algorithm**: Optimal pathfinding with heuristic guidance
- **Dijkstra's Algorithm**: Optimal pathfinding without heuristics
- **TEB Planner**: Timed Elastic Band for dynamic environments
- **RRT-based Planners**: Rapidly-exploring Random Trees

**Local Planner**: Generates safe, executable trajectories in real-time:
- **DWA**: Dynamic Window Approach
- **TebLocalPlanner**: Timed Elastic Band local planner
- **MPC**: Model Predictive Control approaches
- **Reactive Planners**: Simple obstacle avoidance behaviors

**Controller**: Low-level control to execute planned trajectories:
- **Pure Pursuit**: Path following controller
- **PID Controllers**: Proportional-Integral-Derivative control
- **MPC Controllers**: Model predictive control
- **Adaptive Controllers**: Controllers that adjust to robot dynamics

### Humanoid-Specific Navigation Challenges

Humanoid robots face unique navigation challenges that require specialized approaches:

**Bipedal Locomotion**: Unlike wheeled robots, humanoid robots must manage balance while navigating:
- **Dynamic Balance**: Maintaining balance during movement
- **Step Planning**: Planning individual steps for walking
- **ZMP Control**: Zero Moment Point control for stable walking
- **Gait Adaptation**: Adjusting walking patterns for terrain

**Human-Scale Environments**: Navigating spaces designed for human use:
- **Stair Navigation**: Specialized algorithms for stairs
- **Door Navigation**: Techniques for door passage
- **Furniture Navigation**: Navigating around human furniture
- **Narrow Spaces**: Managing navigation in tight spaces

**Social Navigation**: Navigating in human-populated environments:
- **Human-Aware Planning**: Considering human presence in planning
- **Social Norms**: Following social navigation conventions
- **Collision Avoidance**: Avoiding humans safely
- **Interaction Points**: Managing navigation near humans

### Isaac ROS Navigation Enhancements

Isaac ROS enhances Nav2 with GPU-accelerated perception and planning capabilities:

**Accelerated Perception**: Real-time environment understanding:
- **GPU-based Object Detection**: Fast detection of obstacles and humans
- **Semantic Mapping**: Understanding environment semantics
- **Dynamic Object Tracking**: Tracking moving obstacles
- **Scene Understanding**: Real-time scene analysis

**Optimized Planning**: Accelerated path planning algorithms:
- **GPU-based Path Planning**: Accelerated global and local planning
- **Multi-goal Planning**: Planning for multiple objectives
- **Dynamic Replanning**: Real-time plan adjustment
- **Predictive Planning**: Planning considering future states

## Isaac ROS Hardware Acceleration

### GPU Computing for Robotics

Isaac ROS leverages NVIDIA's GPU computing capabilities to accelerate robotics algorithms:

**CUDA Integration**: Direct GPU computing for algorithm acceleration:
- **Parallel Processing**: Massive parallelization of algorithms
- **Memory Bandwidth**: High-bandwidth memory access
- **Specialized Units**: Tensor cores for AI operations
- **Real-time Performance**: Maintaining real-time constraints

**Optimized Libraries**: GPU-accelerated libraries for robotics:
- **NVIDIA VisionWorks**: Computer vision and image processing
- **CUDA-Accelerated OpenCV**: GPU-accelerated computer vision
- **cuDNN**: Deep neural network acceleration
- **TensorRT**: Optimized neural network inference

### TensorRT Integration

TensorRT provides optimized neural network inference for robotics applications:

**Model Optimization**: Optimizing neural networks for deployment:
- **Precision Optimization**: Mixed precision for performance
- **Layer Fusion**: Combining operations for efficiency
- **Memory Optimization**: Reducing memory usage
- **Kernel Optimization**: Optimized GPU kernels

**Real-time Inference**: Maintaining real-time performance:
- **Low Latency**: Minimizing inference time
- **High Throughput**: Processing multiple inputs efficiently
- **Dynamic Batching**: Optimizing batch sizes dynamically
- **Memory Management**: Efficient GPU memory usage

### Isaac ROS Message Types and Interfaces

Isaac ROS provides optimized message types and interfaces:

**Efficient Data Transfer**: Minimizing data transfer overhead:
- **Zero-Copy Transfer**: Direct GPU memory access
- **Batched Processing**: Processing multiple messages efficiently
- **Memory Pools**: Reusing memory allocations
- **Asynchronous Processing**: Non-blocking operations

**Standard ROS 2 Compatibility**: Maintaining compatibility with ROS 2:
- **Standard Message Types**: Compatibility with ROS 2 messages
- **Service Interfaces**: Standard ROS 2 service calls
- **Action Interfaces**: Standard ROS 2 action interfaces
- **Launch System**: Integration with ROS 2 launch

## Advanced Navigation Techniques for Humanoids

### Multi-Modal Navigation

Humanoid robots benefit from using multiple navigation modalities simultaneously:

**Visual-Inertial Navigation**: Combining visual and inertial sensors:
- **VIO Systems**: Visual-Inertial Odometry
- **Loosely Coupled**: Separate processing with fusion
- **Tightly Coupled**: Joint estimation of all states
- **Robust Performance**: Maintaining accuracy in challenging conditions

**LiDAR-Visual Fusion**: Combining LiDAR and visual information:
- **Multi-sensor SLAM**: Simultaneous use of multiple sensors
- **Complementary Information**: Each sensor compensates for others' weaknesses
- **Robust Mapping**: More reliable map building
- **Accurate Localization**: Improved pose estimation

### Dynamic Obstacle Handling

Humanoid robots must navigate around dynamic obstacles including humans:

**Predictive Obstacle Avoidance**: Anticipating obstacle movements:
- **Trajectory Prediction**: Predicting human and object trajectories
- **Probabilistic Planning**: Planning with uncertainty
- **Social Force Models**: Modeling human movement patterns
- **Intent Recognition**: Understanding obstacle intentions

**Reactive Navigation**: Responding to immediate obstacles:
- **Local Replanning**: Real-time path adjustment
- **Velocity Obstacles**: Avoiding collision in velocity space
- **Dynamic Window**: Safe velocity selection
- **Emergency Stops**: Safety-critical stopping

### Terrain-Aware Navigation

Humanoid robots must adapt to different terrain types:

**Terrain Classification**: Identifying traversable terrain:
- **Visual Terrain Analysis**: Using cameras for terrain assessment
- **LiDAR Terrain Mapping**: Using range sensors for terrain analysis
- **Multi-modal Fusion**: Combining sensor information
- **Semantic Segmentation**: Understanding terrain types

**Adaptive Locomotion**: Adjusting walking patterns for terrain:
- **Gait Adaptation**: Changing walking patterns for terrain
- **Step Height Adjustment**: Adapting step height for obstacles
- **Balance Control**: Adjusting balance for terrain
- **Foot Placement**: Optimizing foot placement for stability

## Integration with Humanoid Control Systems

### Coordinated Motion Planning

Navigation and control systems must be tightly integrated for humanoid robots:

**Whole-Body Motion Planning**: Planning motion for entire robot body:
- **Kinematic Constraints**: Respecting joint limits and constraints
- **Dynamic Constraints**: Maintaining balance during motion
- **Collision Avoidance**: Avoiding self-collision and environment collision
- **Task Coordination**: Coordinating multiple tasks simultaneously

**Hierarchical Control**: Multi-level control architecture:
- **High-level Planning**: Task-level planning
- **Mid-level Control**: Trajectory generation
- **Low-level Control**: Joint-level control
- **Safety Systems**: Emergency stop and safety monitoring

### Balance and Navigation Coordination

Maintaining balance while navigating requires careful coordination:

**Balance-Aware Navigation**: Considering balance in navigation planning:
- **Stability Regions**: Planning paths that maintain balance
- **ZMP Planning**: Zero Moment Point trajectory planning
- **Capture Point**: Planning to maintain balance recovery
- **Push Recovery**: Planning for unexpected disturbances

**Dynamic Walking**: Walking patterns that support navigation:
- **Stable Gaits**: Walking patterns that maintain stability
- **Adaptive Timing**: Adjusting step timing for navigation
- **Reactive Adjustments**: Adjusting to unexpected conditions
- **Energy Efficiency**: Optimizing for energy consumption

## Performance Optimization and Real-time Considerations

### Real-time Constraints

Humanoid navigation systems must meet strict real-time requirements:

**Timing Requirements**: Different systems have different timing needs:
- **Control Loop**: Typically 100-1000 Hz for balance control
- **Planning Loop**: 10-50 Hz for path planning
- **Perception Loop**: 10-30 Hz for environment understanding
- **High-level Planning**: 1-5 Hz for route planning

**Latency Management**: Minimizing system latency:
- **Pipeline Optimization**: Reducing processing pipeline latency
- **Memory Management**: Efficient memory allocation and reuse
- **Thread Management**: Proper thread scheduling and synchronization
- **Communication Optimization**: Efficient message passing

### Resource Management

Efficient use of computational resources is crucial:

**GPU Resource Allocation**: Managing GPU resources effectively:
- **Memory Management**: Efficient GPU memory usage
- **Kernel Scheduling**: Proper GPU kernel scheduling
- **Multi-GPU Utilization**: Using multiple GPUs when available
- **Load Balancing**: Balancing computational load

**CPU-GPU Coordination**: Coordinating CPU and GPU processing:
- **Task Scheduling**: Proper scheduling of CPU and GPU tasks
- **Data Transfer**: Minimizing CPU-GPU data transfer
- **Synchronization**: Proper synchronization between components
- **Resource Sharing**: Sharing resources efficiently

## Best Practices for Isaac ROS Navigation

### System Design Principles

Effective Isaac ROS navigation system design follows several principles:

**Modular Architecture**: Building systems with clear interfaces:
- **Component Independence**: Components that can be developed independently
- **Standard Interfaces**: Clear, well-defined interfaces between components
- **Plug-and-Play**: Easy replacement of components
- **Testability**: Components that can be tested independently

**Performance Monitoring**: Continuously monitoring system performance:
- **Real-time Metrics**: Monitoring real-time performance metrics
- **Resource Usage**: Monitoring CPU, GPU, and memory usage
- **Latency Tracking**: Monitoring processing latency
- **Accuracy Metrics**: Tracking navigation accuracy

### Safety and Reliability

Navigation systems for humanoid robots must prioritize safety:

**Safety Architecture**: Building safety into the system:
- **Redundant Systems**: Multiple systems for critical functions
- **Fail-safe Modes**: Safe behavior when systems fail
- **Emergency Procedures**: Defined emergency procedures
- **Safety Monitoring**: Continuous safety system monitoring

**Validation and Testing**: Thoroughly testing navigation systems:
- **Simulation Testing**: Extensive testing in simulation
- **Hardware-in-Loop**: Testing with real hardware components
- **Real-world Testing**: Gradual introduction to real environments
- **Edge Case Testing**: Testing challenging scenarios

## Conclusion

Isaac ROS represents a significant advancement in robotics navigation, providing GPU-accelerated implementations of critical perception and navigation algorithms specifically optimized for humanoid robots. The integration of Isaac ROS with Nav2 creates a powerful platform for developing sophisticated navigation capabilities that can handle the unique challenges of humanoid robot locomotion in complex, dynamic environments.

The combination of hardware acceleration, optimized algorithms, and standard ROS 2 compatibility enables the development of navigation systems that meet the real-time performance requirements essential for humanoid robot safety and responsiveness. As humanoid robotics continues to advance, platforms like Isaac ROS will play an increasingly important role in enabling these systems to operate autonomously in human environments.

The next chapter will explore the integration of voice recognition and large language models for humanoid robot planning and control, representing the cutting edge of embodied artificial intelligence.

---

## References

[1] NVIDIA. (2023). Isaac ROS Documentation. Retrieved from https://docs.nvidia.com/isaac/ros/

[2] NVIDIA. (2023). Navigation2 (Nav2) Integration Guide. Retrieved from https://navigation.ros.org/

[3] Mur-Artal, R., & Tardós, J. D. (2017). ORB-SLAM2: an open-source SLAM system for monocular, stereo, and RGB-D cameras. *IEEE Transactions on Robotics*, 33(5), 1255-1262.

[4] Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics*. Springer.

[5] Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance. *IEEE Robotics & Automation Magazine*, 4(1), 23-33.

## Code Examples

### Example 1: Isaac ROS VSLAM Node Implementation
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class IsaacROSVisualSLAM(Node):
    def __init__(self):
        super().__init__('isaac_ros_vslam')

        # Create quality of service profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            qos_profile
        )

        # Create publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            '/visual_odom',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/visual_pose',
            10
        )

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize SLAM components
        self.prev_frame = None
        self.prev_kp = None
        self.prev_desc = None
        self.current_pose = np.eye(4)
        self.camera_matrix = None
        self.dist_coeffs = None

        # ORB detector and matcher
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Feature matching parameters
        self.min_matches = 10
        self.max_reprojection_error = 3.0

        self.get_logger().info('Isaac ROS Visual SLAM node initialized')

    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera info"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process incoming image for visual odometry"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process frame for SLAM
            self.process_frame(cv_image, msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_frame(self, frame, timestamp):
        """Process a single frame for visual SLAM"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect and compute features
        kp = self.orb.detect(gray, None)
        kp, desc = self.orb.compute(gray, kp)

        if self.prev_frame is not None and self.prev_desc is not None and desc is not None:
            # Match features between current and previous frames
            matches = self.bf.match(self.prev_desc, desc)

            # Sort matches by distance
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) >= self.min_matches:
                # Extract matched keypoints
                src_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

                # Estimate essential matrix
                E, mask = cv2.findEssentialMat(
                    src_pts, dst_pts,
                    self.camera_matrix,
                    method=cv2.RANSAC,
                    threshold=self.max_reprojection_error
                )

                if E is not None:
                    # Recover pose from essential matrix
                    _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix)

                    # Create transformation matrix
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = t.flatten()

                    # Update current pose
                    self.current_pose = self.current_pose @ np.linalg.inv(T)

                    # Publish odometry
                    self.publish_odometry(self.current_pose, timestamp)

        # Update previous frame data
        self.prev_frame = gray.copy()
        self.prev_kp = kp
        self.prev_desc = desc

    def publish_odometry(self, pose_matrix, timestamp):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Extract position and orientation from transformation matrix
        position = pose_matrix[:3, 3]
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]

        # Convert rotation matrix to quaternion
        rotation_matrix = pose_matrix[:3, :3]
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(rotation_matrix)
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Also publish pose stamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = timestamp
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_stamped)

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return qw, qx, qy, qz

def main(args=None):
    rclpy.init(args=args)
    vslam_node = IsaacROSVisualSLAM()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        vslam_node.get_logger().info('Shutting down Visual SLAM node')
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Isaac ROS Navigation Configuration
```xml
<launch>
  <!-- Isaac ROS Navigation Configuration -->
  <arg name="namespace" default="humanoid_robot"/>
  <arg name="use_sim_time" default="false"/>
  <arg name="autostart" default="true"/>
  <arg name="params_file" default="$(find-pkg-share isaac_ros_nav2)/config/nav2_params.yaml"/>

  <!-- Navigation2 Stack -->
  <group>
    <push-ros-namespace namespace="$(var namespace)"/>

    <!-- Map Server -->
    <node pkg="nav2_map_server" exec="map_server" name="map_server">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="yaml_filename" value="map.yaml"/>
    </node>

    <!-- Local Costmap -->
    <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="local_costmap" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="global_frame" value="odom"/>
      <param name="robot_base_frame" value="base_link"/>
      <param name="update_frequency" value="5.0"/>
      <param name="publish_frequency" value="2.0"/>
      <param name="width" value="10.0"/>
      <param name="height" value="10.0"/>
      <param name="resolution" value="0.05"/>
      <param name="origin_x" value="-5.0"/>
      <param name="origin_y" value="-5.0"/>
    </node>

    <!-- Global Costmap -->
    <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="global_costmap" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="global_frame" value="map"/>
      <param name="robot_base_frame" value="base_link"/>
      <param name="update_frequency" value="1.0"/>
      <param name="publish_frequency" value="0.5"/>
      <param name="width" value="40.0"/>
      <param name="height" value="40.0"/>
      <param name="resolution" value="0.05"/>
      <param name="origin_x" value="-20.0"/>
      <param name="origin_y" value="-20.0"/>
    </node>

    <!-- Planner Server -->
    <node pkg="nav2_planner" exec="planner_server" name="planner_server" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="planner_plugins" value="GridBased"/>
      <param name="GridBased.type" value="nav2_navfn_planner/NavfnPlanner"/>
    </node>

    <!-- Controller Server -->
    <node pkg="nav2_controller" exec="controller_server" name="controller_server" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="controller_frequency" value="20.0"/>
      <param name="min_x_velocity_threshold" value="0.001"/>
      <param name="min_y_velocity_threshold" value="0.5"/>
      <param name="min_theta_velocity_threshold" value="0.001"/>
      <param name="progress_checker_plugin" value="progress_checker"/>
      <param name="goal_checker_plugin" value="goal_checker"/>
      <param name="controller_plugins" value="FollowPath"/>
      <param name="FollowPath.type" value="nav2_mppi_controller::MPPIController"/>
    </node>

    <!-- Behavior Tree Server -->
    <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="bt_loop_duration" value="10"/>
      <param name="default_server_timeout" value="20"/>
      <param name="enable_groot_monitoring" value="true"/>
      <param name="groot_zmq_publisher_port" value="1666"/>
      <param name="groot_zmq_server_port" value="1667"/>
    </node>

    <!-- Lifecycle Manager -->
    <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager_navigation">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="autostart" value="$(var autostart)"/>
      <param name="node_names" value="[map_server, local_costmap, global_costmap, planner_server, controller_server, bt_navigator]"/>
    </node>

    <!-- Isaac ROS Perception Nodes -->
    <node pkg="isaac_ros_detectnet" exec="isaac_ros_detectnet" name="detectnet" output="screen">
      <param name="input_topic" value="/camera/rgb/image_raw"/>
      <param name="camera_info_topic" value="/camera/rgb/camera_info"/>
      <param name="tensorrt_engine_file_path" value="detectnet_model.plan"/>
      <param name="model_input_width" value="960"/>
      <param name="model_input_height" value="544"/>
      <param name="max_batch_size" value="1"/>
      <param name="confidence_threshold" value="0.5"/>
    </node>

    <node pkg="isaac_ros_stereo_image_proc" exec="isaac_ros_stereo_rectify" name="stereo_rectify" output="screen">
      <param name="left_topic" value="/camera/left/image_raw"/>
      <param name="right_topic" value="/camera/right/image_raw"/>
      <param name="left_camera_info_topic" value="/camera/left/camera_info"/>
      <param name="right_camera_info_topic" value="/camera/right/camera_info"/>
    </node>

    <node pkg="isaac_ros_pointcloud_utils" exec="isaac_ros_pointcloud_to_laserscan" name="pointcloud_to_laserscan" output="screen">
      <param name="input_topic" value="/points2"/>
      <param name="output_frame" value="base_link"/>
      <param name="range_min" value="0.1"/>
      <param name="range_max" value="20.0"/>
      <param name="scan_height" value="1"/>
    </node>

  </group>
</launch>
```

### Example 3: Isaac ROS Localization Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import numpy as np
import scipy.stats as stats
from collections import deque
import math

class IsaacROSLocalization(Node):
    def __init__(self):
        super().__init__('isaac_ros_localization')

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            10
        )

        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Particle filter parameters
        self.num_particles = 1000
        self.particles = np.zeros((self.num_particles, 3))  # x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Motion model parameters
        self.motion_model_variance = [0.1, 0.1, 0.05]  # x, y, theta

        # Sensor model parameters
        self.laser_sigma_hit = 0.2
        self.laser_lambda_short = 0.1
        self.laser_z_hit = 0.8
        self.laser_z_short = 0.1
        self.laser_z_max = 0.05
        self.laser_z_rand = 0.05

        # Initialize with uniform distribution
        self.initialize_particles()

        # Previous odometry for motion model
        self.prev_odom = None
        self.odom_pose = np.array([0.0, 0.0, 0.0])

        # Map (simplified - in real implementation, this would be loaded from map server)
        self.map_resolution = 0.05
        self.map_origin = [-10.0, -10.0]  # x, y
        self.map_size = [400, 400]  # cells

        self.get_logger().info('Isaac ROS Localization node initialized')

    def initialize_particles(self):
        """Initialize particles with uniform distribution"""
        # For simplicity, initialize around origin
        # In real implementation, this would be based on initial pose or map
        for i in range(self.num_particles):
            self.particles[i, 0] = np.random.uniform(-2.0, 2.0)  # x
            self.particles[i, 1] = np.random.uniform(-2.0, 2.0)  # y
            self.particles[i, 2] = np.random.uniform(-np.pi, np.pi)  # theta

    def scan_callback(self, msg):
        """Process laser scan for particle filter update"""
        if self.prev_odom is None:
            return

        # Predict step: update particles based on odometry
        self.predict_motion()

        # Update step: weight particles based on laser scan
        self.update_weights(msg)

        # Resample particles
        self.resample_particles()

        # Publish estimated pose
        self.publish_estimated_pose()

    def odom_callback(self, msg):
        """Update odometry information"""
        # Extract pose from odometry
        self.odom_pose[0] = msg.pose.pose.position.x
        self.odom_pose[1] = msg.pose.pose.position.y

        # Convert quaternion to euler
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.odom_pose[2] = math.atan2(siny_cosp, cosy_cosp)

        # Store for motion prediction
        self.prev_odom = msg

    def imu_callback(self, msg):
        """Process IMU data for orientation"""
        # Extract orientation from IMU
        quat = msg.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        imu_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Could use IMU to improve orientation estimates
        pass

    def predict_motion(self):
        """Predict particle poses based on odometry motion"""
        if self.prev_odom is None:
            return

        # Calculate motion since last update
        dx = self.odom_pose[0] - self.prev_odom.pose.pose.position.x
        dy = self.odom_pose[1] - self.prev_odom.pose.pose.position.y

        # Convert to polar coordinates
        dist = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)

        # Add noise to motion
        for i in range(self.num_particles):
            # Add noise to distance and angle
            noisy_dist = dist + np.random.normal(0, self.motion_model_variance[0] * dist)
            noisy_angle = angle + np.random.normal(0, self.motion_model_variance[2])

            # Update particle position
            self.particles[i, 0] += noisy_dist * math.cos(self.particles[i, 2] + noisy_angle)
            self.particles[i, 1] += noisy_dist * math.sin(self.particles[i, 2] + noisy_angle)

            # Add noise to orientation
            self.particles[i, 2] += np.random.normal(0, self.motion_model_variance[2])

    def update_weights(self, scan_msg):
        """Update particle weights based on laser scan"""
        for i in range(self.num_particles):
            weight = self.calculate_particle_weight(i, scan_msg)
            self.weights[i] = weight

        # Normalize weights
        total_weight = np.sum(self.weights)
        if total_weight > 0:
            self.weights /= total_weight
        else:
            # If all weights are zero, reset to uniform
            self.weights.fill(1.0 / self.num_particles)

    def calculate_particle_weight(self, particle_idx, scan_msg):
        """Calculate weight for a single particle based on laser scan"""
        particle = self.particles[particle_idx]

        # This is a simplified sensor model
        # In real implementation, this would involve ray tracing through the map
        expected_scan = self.predict_scan(particle, scan_msg)

        # Compare expected scan with actual scan
        weight = 1.0
        for i, (expected, actual) in enumerate(zip(expected_scan, scan_msg.ranges)):
            if not (np.isinf(actual) or np.isnan(actual)):
                # Simple Gaussian comparison
                diff = abs(expected - actual)
                weight *= np.exp(-0.5 * (diff / self.laser_sigma_hit) ** 2)

        return weight

    def predict_scan(self, particle_pose, scan_msg):
        """Predict expected laser scan for a particle (simplified)"""
        # This is a placeholder - real implementation would involve
        # ray tracing from the particle pose through the occupancy map
        expected_scan = [range_val for range_val in scan_msg.ranges]
        return expected_scan

    def resample_particles(self):
        """Resample particles based on weights"""
        # Systematic resampling
        new_particles = np.zeros_like(self.particles)

        # Calculate cumulative weights
        cumulative_weights = np.cumsum(self.weights)

        # Generate random starting point
        r = np.random.uniform(0, 1.0 / self.num_particles)

        i = 0
        for j in range(self.num_particles):
            U = r + j * (1.0 / self.num_particles)
            while cumulative_weights[i] < U and i < self.num_particles - 1:
                i += 1
            new_particles[j] = self.particles[i]

        self.particles = new_particles
        self.weights.fill(1.0 / self.num_particles)

    def publish_estimated_pose(self):
        """Publish estimated robot pose"""
        # Calculate mean pose from particles
        mean_x = np.average(self.particles[:, 0], weights=self.weights)
        mean_y = np.average(self.particles[:, 1], weights=self.weights)

        # For orientation, need to handle angle wrapping
        sin_avg = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        cos_avg = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        mean_theta = math.atan2(sin_avg, cos_avg)

        # Calculate covariance
        cov_x = np.average((self.particles[:, 0] - mean_x)**2, weights=self.weights)
        cov_y = np.average((self.particles[:, 1] - mean_y)**2, weights=self.weights)

        # Create and publish pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = mean_x
        pose_msg.pose.pose.position.y = mean_y
        pose_msg.pose.pose.position.z = 0.0

        # Convert orientation to quaternion
        qw = math.cos(mean_theta / 2)
        qz = math.sin(mean_theta / 2)
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.pose.orientation.z = qz

        # Set covariance
        pose_msg.pose.covariance[0] = cov_x  # x
        pose_msg.pose.covariance[7] = cov_y  # y
        pose_msg.pose.covariance[35] = 0.1  # theta (approximate)

        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    localization_node = IsaacROSLocalization()

    try:
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        localization_node.get_logger().info('Shutting down localization node')
    finally:
        localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 4: Isaac ROS Navigation Server Configuration
```yaml
# Isaac ROS Navigation Configuration
isaac_ros_nav2:
  ros__parameters:
    use_sim_time: false

    # Controller server configuration
    controller_server:
      ros__parameters:
        use_sim_time: false
        controller_frequency: 20.0
        min_x_velocity_threshold: 0.001
        min_y_velocity_threshold: 0.5
        min_theta_velocity_threshold: 0.001
        progress_checker_plugin: "progress_checker"
        goal_checker_plugin: "goal_checker"
        controller_plugins: ["FollowPath"]

        # DWB Controller
        FollowPath:
          plugin: "dwb_core::DWBLocalPlanner"
          debug_trajectory_details: True
          min_vel_x: 0.0
          min_vel_y: 0.0
          max_vel_x: 0.5
          max_vel_y: 0.0
          max_vel_theta: 1.0
          min_speed_xy: 0.0
          max_speed_xy: 0.5
          min_speed_theta: 0.0
          acc_lim_x: 2.5
          acc_lim_y: 0.0
          acc_lim_theta: 3.2
          decel_lim_x: -2.5
          decel_lim_y: 0.0
          decel_lim_theta: -3.2
          vx_samples: 20
          vy_samples: 0
          vtheta_samples: 40
          sim_time: 1.7
          linear_granularity: 0.05
          angular_granularity: 0.025
          transform_tolerance: 0.2
          xy_goal_tolerance: 0.25
          yaw_goal_tolerance: 0.25
          stateful: True
          restore_defaults: False

    # Planner server configuration
    planner_server:
      ros__parameters:
        use_sim_time: false
        planner_plugins: ["GridBased"]
        GridBased:
          plugin: "nav2_navfn_planner/NavfnPlanner"
          tolerance: 0.5
          use_astar: false
          allow_unknown: true

    # Recovery server configuration
    recovery_server:
      ros__parameters:
        use_sim_time: false
        recovery_plugins: ["spin", "backup", "wait"]
        spin:
          plugin: "nav2_recoveries/Spin"
          sim_frequency: 10
          linear_acc_lim: 0.5
          max_rotation_attempts: 10
          rotation_speed: 1.0
          min_duration: 0.5
          max_duration: 10.0
        backup:
          plugin: "nav2_recoveries/BackUp"
          sim_frequency: 10
          translation_weight: 10.0
          rotation_weight: 1.0
          scaling_weight: 1.0
          min_duration: 1.0
          max_duration: 10.0
        wait:
          plugin: "nav2_recoveries/Wait"
          sim_frequency: 10
          backup_dist: 0.15
          min_duration: 1.0
          max_duration: 10.0

    # BT Navigator configuration
    bt_navigator:
      ros__parameters:
        use_sim_time: false
        global_frame: "map"
        robot_base_frame: "base_link"
        odom_topic: "/odom"
        default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
        plugin_lib_names:
          - "nav2_compute_path_to_pose_action_bt_node"
          - "nav2_follow_path_action_bt_node"
          - "nav2_back_up_action_bt_node"
          - "nav2_spin_action_bt_node"
          - "nav2_wait_action_bt_node"
          - "nav2_clear_costmap_service_bt_node"
          - "nav2_is_stuck_condition_bt_node"
          - "nav2_goal_reached_condition_bt_node"
          - "nav2_goal_updated_condition_bt_node"
          - "nav2_initial_pose_received_condition_bt_node"
          - "nav2_reinitialize_global_localization_service_bt_node"
          - "nav2_rate_controller_bt_node"
          - "nav2_distance_controller_bt_node"
          - "nav2_speed_controller_bt_node"
          - "nav2_truncate_path_action_bt_node"
          - "nav2_goal_updater_node_bt_node"
          - "nav2_recovery_node_bt_node"
          - "nav2_pipeline_sequence_bt_node"
          - "nav2_round_robin_node_bt_node"
          - "nav2_transform_available_condition_bt_node"
          - "nav2_time_expired_condition_bt_node"
          - "nav2_path_expiring_timer_condition"
          - "nav2_distance_traveled_condition_bt_node"
          - "nav2_single_trigger_bt_node"
          - "nav2_is_battery_low_condition_bt_node"
          - "nav2_navigate_through_poses_action_bt_node"
          - "nav2_navigate_to_pose_action_bt_node"
          - "nav2_remove_passed_goals_action_bt_node"
          - "nav2_planner_selector_bt_node"
          - "nav2_controller_selector_bt_node"
          - "nav2_goal_checker_selector_bt_node"
          - "nav2_controller_cancel_bt_node"
          - "nav2_path_longer_on_approach_bt_node"
          - "nav2_wait_cancel_bt_node"

    # Local costmap configuration
    local_costmap:
      ros__parameters:
        update_frequency: 5.0
        publish_frequency: 2.0
        global_frame: "odom"
        robot_base_frame: "base_link"
        use_sim_time: false
        rolling_window: true
        width: 10
        height: 10
        resolution: 0.05
        robot_radius: 0.3
        plugins: ["voxel_layer", "inflation_layer"]
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55
        voxel_layer:
          plugin: "nav2_costmap_2d::VoxelLayer"
          enabled: True
          publish_voxel_map: True
          origin_z: 0.0
          z_resolution: 0.2
          z_voxels: 10
          max_obstacle_height: 2.0
          mark_threshold: 0
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0

    # Global costmap configuration
    global_costmap:
      ros__parameters:
        update_frequency: 1.0
        publish_frequency: 0.5
        global_frame: "map"
        robot_base_frame: "base_link"
        use_sim_time: false
        robot_radius: 0.3
        resolution: 0.05
        track_unknown_space: true
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: True
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
        static_layer:
          plugin: "nav2_costmap_2d::StaticLayer"
          map_subscribe_transient_local: True
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55
```

### Example 5: Isaac ROS Humanoid Navigation Interface
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point
import numpy as np
import math

class IsaacROSHumanoidNavigator(Node):
    def __init__(self):
        super().__init__('isaac_ros_humanoid_navigator')

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/humanoid_path', 10)

        # Navigation parameters
        self.navigation_active = False
        self.safety_distance = 0.5  # meters
        self.max_linear_speed = 0.3  # m/s for humanoid
        self.max_angular_speed = 0.5  # rad/s

        # Obstacle avoidance
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self.obstacle_distance = float('inf')

        self.get_logger().info('Isaac ROS Humanoid Navigator initialized')

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        qw = math.cos(theta / 2)
        qz = math.sin(theta / 2)
        goal_msg.pose.pose.orientation.w = qw
        goal_msg.pose.pose.orientation.z = qz

        # Send goal
        self.get_logger().info(f'Sending navigation goal to ({x}, {y}, {theta})')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Navigation goal accepted')
        self.navigation_active = True

        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.navigation_active = False

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Find minimum distance in front of robot
        front_scan_start = len(msg.ranges) // 2 - 30  # ~30 degrees left of center
        front_scan_end = len(msg.ranges) // 2 + 30    # ~30 degrees right of center

        min_distance = float('inf')
        min_angle = 0

        for i in range(front_scan_start, front_scan_end):
            if i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]):
                if msg.ranges[i] < min_distance:
                    min_distance = msg.ranges[i]
                    min_angle = msg.angle_min + i * msg.angle_increment

        self.obstacle_distance = min_distance
        self.obstacle_angle = min_angle
        self.obstacle_detected = min_distance < self.safety_distance

        # If navigating and obstacle detected, slow down or stop
        if self.navigation_active and self.obstacle_detected:
            self.handle_obstacle()

    def handle_obstacle(self):
        """Handle obstacle detection during navigation"""
        self.get_logger().warn(f'Obstacle detected at {self.obstacle_distance:.2f}m, angle {math.degrees(self.obstacle_angle):.1f}°')

        # Stop robot or slow down
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        # If obstacle is very close, cancel navigation
        if self.obstacle_distance < 0.3:
            self.cancel_navigation()

    def cancel_navigation(self):
        """Cancel current navigation"""
        # In a real implementation, you would cancel the goal
        # For now, just set the flag
        self.navigation_active = False
        self.get_logger().info('Navigation cancelled due to obstacle')

    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = min(linear_x, self.max_linear_speed)
        cmd_vel.angular.z = min(angular_z, self.max_angular_speed)
        self.cmd_vel_pub.publish(cmd_vel)

    def get_navigation_status(self):
        """Get current navigation status"""
        return {
            'navigation_active': self.navigation_active,
            'obstacle_detected': self.obstacle_detected,
            'obstacle_distance': self.obstacle_distance,
            'obstacle_angle': self.obstacle_angle
        }

def main(args=None):
    rclpy.init(args=args)
    navigator = IsaacROSHumanoidNavigator()

    # Example: Navigate to a specific location
    navigator.send_navigation_goal(5.0, 3.0, 0.0)  # x=5m, y=3m, theta=0

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down navigator')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```