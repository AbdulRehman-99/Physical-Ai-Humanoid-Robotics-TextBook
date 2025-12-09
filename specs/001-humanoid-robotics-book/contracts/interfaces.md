# Conceptual Interfaces for Humanoid Robotics & Embodied Intelligence

This document outlines the conceptual interfaces and interaction patterns between various components discussed in the book. While not formal API schemas, these define the expected inputs, outputs, and behaviors for key system integrations.

## 1. ROS 2 Control Interface

*   **Description**: Defines the communication patterns for controlling humanoid robots via ROS 2.
*   **Input**:
    *   `/cmd_vel` (geometry_msgs/Twist): Velocity commands for base movement (if applicable).
    *   `/joint_commands` (sensor_msgs/JointState or custom message): Desired joint positions, velocities, or efforts.
    *   `/robot_actions` (custom action message): High-level action goals (e.g., "walk_forward", "wave").
*   **Output**:
    *   `/joint_states` (sensor_msgs/JointState): Current joint positions, velocities, and efforts.
    *   `/odom` (nav_msgs/Odometry): Robot's pose estimate and velocity.
    *   `/tf` (tf2/tfMessage): Transformation frames for robot links and sensors.
*   **Behavior**: ROS 2 nodes subscribe to command topics/actions and publish state information. Controllers translate high-level commands into low-level joint actuation.

## 2. Simulation Environment Interface (Gazebo/Unity/Isaac Sim)

*   **Description**: Defines how the robot control stack interacts with the simulation environments.
*   **Input**:
    *   Robot control commands (via ROS 2 bridge or direct API calls).
    *   World/scene descriptions (URDF, SDF, Unity scene files).
*   **Output**:
    *   Simulated sensor data (camera images, LiDAR scans, IMU data).
    *   Ground truth robot pose and joint states.
    *   Physics engine feedback (collisions, contact forces).
*   **Behavior**: Simulations receive control inputs, update robot state based on physics, and generate sensor outputs that mimic real-world sensors.

## 3. Perception System Interface (Isaac ROS, VSLAM)

*   **Description**: Defines the inputs and outputs for environmental perception and mapping modules.
*   **Input**:
    *   `sensor_msgs/Image`: Raw camera images.
    *   `sensor_msgs/PointCloud2`: LiDAR or depth camera point clouds.
    *   `sensor_msgs/Imu`: Inertial Measurement Unit data.
*   **Output**:
    *   `nav_msgs/OccupancyGrid`: 2D or 3D occupancy maps.
    *   `geometry_msgs/PoseStamped`: Robot's estimated pose.
    *   `sensor_msgs/PointCloud2`: Mapped features or environment point clouds.
    *   `visualization_msgs/MarkerArray`: For debugging and visualization of features/waypoints.
*   **Behavior**: Processes raw sensor data to build an understanding of the environment, including localization (VSLAM) and mapping.

## 4. Navigation Stack Interface (Nav2)

*   **Description**: Defines the interaction with the autonomous navigation system.
*   **Input**:
    *   `nav_msgs/OccupancyGrid`: Environment map.
    *   `geometry_msgs/PoseStamped`: Goal pose.
    *   `geometry_msgs/PoseWithCovarianceStamped`: Initial pose estimate.
    *   Robot odometry and sensor data (via ROS 2 topics).
*   **Output**:
    *   `geometry_msgs/Twist`: Velocity commands for robot movement.
    *   `nav_msgs/Path`: Planned path.
    *   `actionlib_msgs/GoalStatusArray`: Status of navigation goals.
*   **Behavior**: Receives a goal, plans a path, and generates velocity commands to guide the robot while avoiding obstacles, using the environment map.

## 5. Vision-Language-Action (VLA) System Interface

*   **Description**: Defines the interaction flow for natural language commanding and intelligent task execution.
*   **Input**:
    *   `std_msgs/String`: Natural language text commands (e.g., from Whisper ASR).
    *   `sensor_msgs/Image`: Visual input for object detection/scene understanding.
    *   Current robot state (joint states, pose).
*   **Output**:
    *   `robot_action_msgs/ActionPlan`: A sequence of ROS 2 actions or atomic commands.
    *   `std_msgs/String`: Textual feedback or clarification requests.
    *   `visualization_msgs/MarkerArray`: Visual cues for detected objects or planned interactions.
*   **Behavior**: The VLA system interprets natural language, uses visual input to understand the scene, reasons about available robot capabilities, and generates a plan of ROS 2 actions to achieve the desired outcome.

## 6. Docusaurus Content Interface

*   **Description**: Defines the structure and expected format for content to be consumed by Docusaurus.
*   **Input**:
    *   Markdown (`.md`, `.mdx`) files for chapters and pages.
    *   `sidebars.js`: Configuration for navigation.
    *   `docusaurus.config.js`: Site-wide configuration.
    *   Static assets (images, videos) in `static/` directory.
*   **Output**:
    *   Generated static HTML website.
*   **Behavior**: Docusaurus processes markdown and configuration files to build a navigable, searchable, and versioned technical book website.
