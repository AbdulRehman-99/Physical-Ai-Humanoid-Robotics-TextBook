# Data Model: Technical Book: Humanoid Robotics & Embodied Intelligence

This document outlines the key entities and their relationships relevant to the content and examples presented in the technical book.

## Entities

### 1. Humanoid Robot Model
**Description**: Represents the physical and simulated robot, including its kinematic structure, dynamic properties, and sensor configurations.
**Fields**:
- `id`: Unique identifier (e.g., string for model name)
- `name`: Human-readable name (string)
- `type`: (e.g., "bipedal", "humanoid") (string)
- `urdf_path`: Path to URDF/Xacro definition file (string)
- `kinematics`: Description of joint chains, degrees of freedom
- `dynamics`: Mass, inertia properties
- `sensors`: List of attached sensors (e.g., camera, lidar, IMU)

### 2. Simulation Environment
**Description**: Digital twin platforms used for testing, development, and visualizing robot behavior.
**Fields**:
- `id`: Unique identifier (e.g., string for environment name)
- `name`: Human-readable name (string)
- `platform`: (e.g., "Gazebo", "Unity", "Isaac Sim") (string)
- `version`: Software version of the simulation platform (string)
- `assets`: List of objects, terrains, and robot models within the environment
- `scale`: Typical dimensions/size (e.g., 5x5m, 10x10m for SLAM environments)

### 3. Sensor Data
**Description**: Inputs from various robot sensors used for perception and environmental understanding.
**Fields**:
- `sensor_type`: (e.g., "camera", "lidar", "IMU") (string)
- `data_format`: (e.g., "image", "point cloud", "IMU message") (string)
- `timestamp`: Time of data capture (datetime)
- `value`: Raw or processed sensor readings
- `source_robot_id`: Reference to the `Humanoid Robot Model` providing the data
- `typical_volume`: Scenes with 5-10 distinct objects (for VLA examples)

### 4. Control Commands
**Description**: Instructions sent to the robot to dictate its actions and movements.
**Fields**:
- `id`: Unique command identifier (e.g., UUID)
- `type`: (e.g., "joint_position", "velocity", "path_plan", "high_level_action") (string)
- `target_robot_id`: Reference to the `Humanoid Robot Model` to be controlled
- `parameters`: Command-specific arguments (e.g., joint angles, target pose, movement speed)
- `source_user_id`: Identifier for the student/user issuing the command

### 5. Environment Map
**Description**: Representation of the robot's surroundings, built via Simultaneous Localization and Mapping (SLAM).
**Fields**:
- `id`: Unique map identifier
- `name`: Human-readable map name
- `type`: (e.g., "occupancy grid", "point cloud map", "mesh map") (string)
- `resolution`: Spatial resolution of the map (float)
- `creation_timestamp`: Time of map creation (datetime)
- `bounding_box`: Spatial extent of the map (e.g., min/max coordinates)
- `source_robot_id`: Reference to the `Humanoid Robot Model` that generated the map
- `typical_size`: Room sizes of 5x5m to 10x10m

### 6. Natural Language Commands
**Description**: Text-based instructions given to the Vision-Language-Action (VLA) system for high-level robot control.
**Fields**:
- `id`: Unique command identifier
- `text`: The natural language command string (e.g., "pick up the red block") (string)
- `intent`: Inferred action or goal (string)
- `entities`: Identified objects or locations from the command (list of strings)
- `command_target_id`: Reference to the `Humanoid Robot Model`
- `timestamp`: Time of command issuance

### 7. Course Module
**Description**: A defined unit of learning that the book content will map to, providing structure for incremental learning.
**Fields**:
- `id`: Unique module identifier
- `title`: Module title (string)
- `description`: Brief overview of module content
- `learning_outcomes`: List of expected student achievements
- `chapters`: List of book chapters mapped to this module

## Relationships

-   `Humanoid Robot Model` is used within `Simulation Environment`s.
-   `Humanoid Robot Model` generates `Sensor Data`.
-   `Humanoid Robot Model` is controlled by `Control Commands` and `Natural Language Commands`.
-   `Sensor Data` is used to build `Environment Map`s.
-   `Natural Language Commands` are processed by a VLA system, which generates `Control Commands`.
-   `Course Module`s contain `Book Chapters` (implicit through book structure).