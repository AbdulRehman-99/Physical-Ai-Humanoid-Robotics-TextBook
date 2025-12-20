# Data Model: Physical AI & Humanoid Robotics Technical Book

## Overview
This document defines the data models and entities that will be used throughout the technical book on Physical AI & Humanoid Robotics. These models represent the core concepts that will be explained, demonstrated, and implemented across the 8 chapters and 4 modules.

## Core Entities

### 1. Book Chapter
**Description**: Represents a single chapter in the technical book
**Fields**:
- id: String (unique identifier, e.g., "ch1-foundations-humanoid-basics")
- title: String (chapter title)
- module_id: String (reference to the parent module)
- word_count: Integer (minimum 2500)
- headings: Array of Strings (main headings in the chapter)
- subheadings: Array of Strings (subheadings in the chapter)
- paragraphs: Array of Strings (content paragraphs)
- examples: Array of CodeExample objects
- references: Array of Reference objects
- learning_outcomes: Array of Strings (what students should learn)
- prerequisites: Array of String (required knowledge/skills)

**Validation Rules**:
- word_count must be >= 2500
- title must not be empty
- at least one heading required
- at least one example required
- at least one learning outcome required

### 2. Module
**Description**: Represents a learning module containing 2 chapters
**Fields**:
- id: String (unique identifier, e.g., "module-1-ros")
- title: String (module title)
- chapter_ids: Array of String (references to chapters in this module)
- description: String (module overview)
- learning_path: String (prerequisite path for this module)

**Validation Rules**:
- chapter_ids must contain exactly 2 chapters
- title must not be empty
- description must not be empty

### 3. CodeExample
**Description**: Represents a code example within a chapter
**Fields**:
- id: String (unique identifier)
- language: String (e.g., "python", "cpp", "bash", "yaml")
- code: String (the actual code)
- explanation: String (description of what the code does)
- file_path: String (where the code is stored in the repository)
- simulation_environment: String (e.g., "gazebo", "unity", "isaac-sim", "none")
- dependencies: Array of String (required packages/libraries)

**Validation Rules**:
- language must be one of the supported languages
- code must not be empty
- explanation must not be empty

### 4. Reference
**Description**: Represents a citation or reference in IEEE format
**Fields**:
- id: String (unique identifier)
- type: String (e.g., "book", "journal", "conference", "web", "standard", "manual")
- title: String (title of the reference)
- authors: Array of String (authors of the work)
- publication: String (journal, conference, or publisher name)
- year: Integer (publication year)
- doi: String (optional, for journal articles)
- url: String (optional, for web resources)
- ieee_citation: String (formatted IEEE citation)

**Validation Rules**:
- type must be one of the supported types
- title must not be empty
- authors must not be empty
- year must be a valid year (1900-2030)

### 5. URDFModel
**Description**: Represents a humanoid robot model in URDF format
**Fields**:
- id: String (unique identifier)
- name: String (model name)
- degrees_of_freedom: Integer (typically 28-32 for humanoid)
- joint_configurations: Array of Joint objects
- link_configurations: Array of Link objects
- file_path: String (path to URDF file)
- simulation_compatible: Boolean (whether it works in Gazebo/Unity)
- controllers: Array of String (supported controller types)

**Validation Rules**:
- degrees_of_freedom must be between 28 and 32
- joint_configurations must not be empty
- link_configurations must not be empty

### 6. Joint
**Description**: Represents a single joint in a URDF model
**Fields**:
- name: String (unique name within the model)
- type: String (e.g., "revolute", "prismatic", "fixed", "continuous")
- parent_link: String (name of parent link)
- child_link: String (name of child link)
- axis: Array of Float (3D vector for joint axis)
- limits: JointLimits object (optional, for revolute joints)

**Validation Rules**:
- name must not be empty
- type must be one of the supported types
- parent_link and child_link must not be empty

### 7. JointLimits
**Description**: Represents limits for a joint (for revolute joints)
**Fields**:
- lower: Float (lower limit in radians)
- upper: Float (upper limit in radians)
- effort: Float (maximum effort)
- velocity: Float (maximum velocity)

**Validation Rules**:
- lower must be less than upper
- effort and velocity must be positive

### 8. Link
**Description**: Represents a single link in a URDF model
**Fields**:
- name: String (unique name within the model)
- visual: Visual object (visual representation)
- collision: Collision object (collision representation)
- inertial: Inertial object (physical properties)

**Validation Rules**:
- name must not be empty

### 9. SimulationEnvironment
**Description**: Represents a simulation environment configuration
**Fields**:
- id: String (unique identifier)
- name: String (e.g., "gazebo", "unity", "isaac-sim")
- version: String (version of the simulation software)
- world_config: String (configuration file path)
- physics_properties: PhysicsProperties object
- sensors: Array of Sensor objects

**Validation Rules**:
- name must not be empty
- version must not be empty

### 10. PhysicsProperties
**Description**: Represents physics properties for a simulation environment
**Fields**:
- gravity: Array of Float (3D gravity vector, typically [0, 0, -9.81])
- time_step: Float (simulation time step)
- solver_type: String (e.g., "ode", "bullet")
- real_time_factor: Float (how fast simulation runs relative to real time)

**Validation Rules**:
- gravity must be a 3-element array
- time_step must be positive
- real_time_factor must be positive

### 11. Sensor
**Description**: Represents a sensor in a simulation or real robot
**Fields**:
- id: String (unique identifier)
- type: String (e.g., "camera", "lidar", "imu", "force_torque")
- name: String (sensor name)
- parent_link: String (link to which sensor is attached)
- position: Array of Float (3D position relative to parent)
- orientation: Array of Float (quaternion: [x, y, z, w])
- parameters: Object (sensor-specific parameters)

**Validation Rules**:
- type must be one of the supported sensor types
- name must not be empty
- parent_link must not be empty

### 12. NavigationGoal
**Description**: Represents a navigation goal for the humanoid robot
**Fields**:
- id: String (unique identifier)
- goal_position: Array of Float (3D position [x, y, z])
- goal_orientation: Array of Float (quaternion [x, y, z, w])
- frame_id: String (coordinate frame, e.g., "map", "odom")
- tolerance: Float (acceptable distance to goal)
- timeout: Integer (seconds before goal is considered failed)

**Validation Rules**:
- goal_position must be a 3-element array
- goal_orientation must be a 4-element quaternion array
- tolerance must be positive

### 13. PerceptionTask
**Description**: Represents a perception task for the humanoid robot
**Fields**:
- id: String (unique identifier)
- task_type: String (e.g., "object_detection", "slam", "semantic_segmentation")
- input_source: String (e.g., "camera", "lidar", "imu")
- expected_output: String (description of expected output)
- algorithm: String (e.g., "yolo", "orb_slam", "custom")
- performance_metrics: PerformanceMetrics object

**Validation Rules**:
- task_type must be one of the supported types
- input_source must not be empty
- expected_output must not be empty

### 14. PerformanceMetrics
**Description**: Represents performance metrics for various systems
**Fields**:
- accuracy: Float (0.0-1.0, where 1.0 is perfect)
- precision: Float (0.0-1.0)
- recall: Float (0.0-1.0)
- f1_score: Float (0.0-1.0)
- processing_time_ms: Float (time to process one frame/input)
- throughput_fps: Float (frames per second processed)

**Validation Rules**:
- all metrics must be between 0.0 and 1.0 if applicable
- processing_time_ms must be positive
- throughput_fps must be positive

### 15. VLACommand
**Description**: Represents a Vision-Language-Action command
**Fields**:
- id: String (unique identifier)
- natural_language: String (user's natural language command)
- parsed_action: String (parsed robot action)
- vision_input: String (what the robot sees)
- execution_plan: Array of String (sequence of robot actions)
- confidence: Float (0.0-1.0, confidence in the command interpretation)

**Validation Rules**:
- natural_language must not be empty
- parsed_action must not be empty
- confidence must be between 0.0 and 1.0

## Original Entities from Previous Planning

### 16. Humanoid Robot Model
**Description**: Represents the physical and simulated robot, including its kinematic structure, dynamic properties, and sensor configurations.
**Fields**:
- `id`: Unique identifier (e.g., string for model name)
- `name`: Human-readable name (string)
- `type`: (e.g., "bipedal", "humanoid") (string)
- `urdf_path`: Path to URDF/Xacro definition file (string)
- `kinematics`: Description of joint chains, degrees of freedom
- `dynamics`: Mass, inertia properties
- `sensors`: List of attached sensors (e.g., camera, lidar, IMU)

### 17. Simulation Environment
**Description**: Digital twin platforms used for testing, development, and visualizing robot behavior.
**Fields**:
- `id`: Unique identifier (e.g., string for environment name)
- `name`: Human-readable name (string)
- `platform`: (e.g., "Gazebo", "Unity", "Isaac Sim") (string)
- `version`: Software version of the simulation platform (string)
- `assets`: List of objects, terrains, and robot models within the environment
- `scale`: Typical dimensions/size (e.g., 5x5m, 10x10m for SLAM environments)

### 18. Sensor Data
**Description**: Inputs from various robot sensors used for perception and environmental understanding.
**Fields**:
- `sensor_type`: (e.g., "camera", "lidar", "IMU") (string)
- `data_format`: (e.g., "image", "point cloud", "IMU message") (string)
- `timestamp`: Time of data capture (datetime)
- `value`: Raw or processed sensor readings
- `source_robot_id`: Reference to the `Humanoid Robot Model` providing the data
- `typical_volume`: Scenes with 5-10 distinct objects (for VLA examples)

### 19. Control Commands
**Description**: Instructions sent to the robot to dictate its actions and movements.
**Fields**:
- `id`: Unique command identifier (e.g., UUID)
- `type`: (e.g., "joint_position", "velocity", "path_plan", "high_level_action") (string)
- `target_robot_id`: Reference to the `Humanoid Robot Model` to be controlled
- `parameters`: Command-specific arguments (e.g., joint angles, target pose, movement speed)
- `source_user_id`: Identifier for the student/user issuing the command

### 20. Environment Map
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

### 21. Natural Language Commands
**Description**: Text-based instructions given to the Vision-Language-Action (VLA) system for high-level robot control.
**Fields**:
- `id`: Unique command identifier
- `text`: The natural language command string (e.g., "pick up the red block") (string)
- `intent`: Inferred action or goal (string)
- `entities`: Identified objects or locations from the command (list of strings)
- `command_target_id`: Reference to the `Humanoid Robot Model`
- `timestamp`: Time of command issuance

### 22. Course Module
**Description**: A defined unit of learning that the book content will map to, providing structure for incremental learning.
**Fields**:
- `id`: Unique module identifier
- `title`: Module title (string)
- `description`: Brief overview of module content
- `learning_outcomes`: List of expected student achievements
- `chapters`: List of book chapters mapped to this module

## Relationships

### Book Structure Relationships
- Module 1..* contains 2..2 BookChapter
- BookChapter 1..* uses 0..* CodeExample
- BookChapter 1..* references 0..* Reference

### Robot Model Relationships
- URDFModel 1..* contains 1..* Joint
- URDFModel 1..* contains 1..* Link
- Joint 0..1 has 0..1 JointLimits
- Link 1..1 has 1..1 Visual
- Link 1..1 has 1..1 Collision
- Link 1..1 has 1..1 Inertial

### Simulation Relationships
- SimulationEnvironment 1..* uses 0..* Sensor
- SimulationEnvironment 1..1 has 1..1 PhysicsProperties
- Sensor 0..* attached to 1..1 Link

### Navigation and Perception Relationships
- NavigationGoal 1..1 associated with 1..1 Robot (through URDFModel)
- PerceptionTask 1..1 associated with 1..1 Sensor
- VLACommand 1..1 associated with 1..1 NavigationGoal or PerceptionTask

### Original Relationships from Previous Planning
-   `Humanoid Robot Model` is used within `Simulation Environment`s.
-   `Humanoid Robot Model` generates `Sensor Data`.
-   `Humanoid Robot Model` is controlled by `Control Commands` and `Natural Language Commands`.
-   `Sensor Data` is used to build `Environment Map`s.
-   `Natural Language Commands` are processed by a VLA system, which generates `Control Commands`.
-   `Course Module`s contain `Book Chapters` (implicit through book structure).

## State Transitions

### BookChapter States
- DRAFT → REVIEW → APPROVED → PUBLISHED
- PUBLISHED → REVISION_REQUESTED → DRAFT (if major changes needed)

### Simulation States
- CONFIGURING → RUNNING → PAUSED → STOPPED
- RUNNING → ERROR (if simulation fails)

### Robot Control States
- IDLE → ACTIVE → EXECUTING → COMPLETED/FAILED
- ACTIVE → SAFETY_STOP (if safety conditions triggered)