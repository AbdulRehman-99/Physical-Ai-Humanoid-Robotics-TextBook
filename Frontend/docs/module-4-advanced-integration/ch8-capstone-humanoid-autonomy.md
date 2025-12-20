---
title: "Chapter 8: Capstone - Humanoid Autonomy"
sidebar_position: 2
---

# Chapter 8: Capstone - Humanoid Autonomy

## Introduction to Humanoid Autonomy Systems

Humanoid autonomy represents the ultimate integration of all the technologies and concepts covered throughout this book, combining perception, planning, control, navigation, and interaction capabilities into a cohesive system that can operate independently in complex human environments. True humanoid autonomy encompasses not just the ability to execute predefined tasks, but the capacity for adaptive behavior, learning from experience, and making decisions in uncertain and dynamic environments. This capstone chapter synthesizes the knowledge from all previous chapters to create a comprehensive framework for humanoid robot autonomy.

The challenge of humanoid autonomy lies in the integration of diverse subsystems that must work together seamlessly to achieve complex goals. Unlike simpler robotic platforms, humanoid robots must manage multiple simultaneous objectives: maintaining balance while navigating, perceiving and interacting with the environment, communicating with humans, and executing complex manipulation tasks. This requires sophisticated coordination between perception, planning, control, and learning systems that can adapt to changing conditions and learn from experience.

Humanoid autonomy systems must operate across multiple timescales and levels of abstraction. At the highest level, the system must understand and pursue long-term goals that may span hours or days. At the mid-level, it must plan and execute tasks that take minutes to complete. At the lowest level, it must maintain balance and execute precise movements in real-time. Each level must be aware of the others and coordinate appropriately to achieve coherent behavior.

### Levels of Autonomy in Humanoid Robotics

The development of humanoid autonomy can be understood through several levels of capability:

**Level 1 - Reactive Autonomy**: The robot responds to immediate stimuli and executes pre-programmed behaviors. This level includes basic obstacle avoidance, simple navigation, and predefined manipulation tasks. The robot operates in a stimulus-response mode with limited ability to adapt to novel situations.

**Level 2 - Deliberative Autonomy**: The robot can plan and execute complex tasks by breaking them down into subtasks and reasoning about the best sequence of actions. This level includes task planning, path planning, and basic learning from experience. The robot can adapt its behavior based on environmental conditions but follows predetermined strategies.

**Level 3 - Adaptive Autonomy**: The robot can learn from experience and adapt its strategies over time. This level includes machine learning, behavior adaptation, and the ability to handle novel situations by generalizing from previous experiences. The robot can modify its behavior based on success/failure feedback.

**Level 4 - Collaborative Autonomy**: The robot can work effectively with humans and other robots in shared environments. This level includes human-robot interaction, social navigation, and collaborative task execution. The robot understands social norms and can adapt its behavior to work effectively with humans.

**Level 5 - Cognitive Autonomy**: The robot possesses advanced reasoning capabilities, can set its own goals, and can engage in complex decision-making processes. This level includes high-level reasoning, goal setting, and the ability to operate in completely unstructured environments with minimal human oversight.

### Challenges in Achieving Humanoid Autonomy

Creating truly autonomous humanoid robots presents several fundamental challenges:

**Real-time Performance**: All autonomy systems must operate within strict real-time constraints to maintain safety and responsiveness. This includes perception systems that must process sensor data in real-time, planning systems that must generate paths quickly, and control systems that must maintain balance continuously.

**Uncertainty Management**: Real-world environments are inherently uncertain, with sensor noise, dynamic obstacles, and unpredictable human behavior. Autonomous systems must reason effectively under uncertainty and make robust decisions.

**Multi-objective Optimization**: Humanoid robots must simultaneously optimize multiple competing objectives such as task completion, safety, energy efficiency, and social acceptability. Balancing these objectives requires sophisticated decision-making frameworks.

**Learning and Adaptation**: Autonomous systems must continuously learn from experience and adapt to changing environments, tasks, and user preferences. This requires lifelong learning capabilities and the ability to generalize from limited experience.

## Integration of Perception and Action Systems

### Sensor Fusion for Autonomous Operation

Autonomous humanoid robots must integrate information from multiple sensors to build a coherent understanding of their environment and state. This sensor fusion process combines data from cameras, lidar, IMUs, joint encoders, and other sensors to create a comprehensive representation of the world that supports autonomous decision-making.

**Multi-modal Perception**: The integration of different sensor modalities provides complementary information that improves the robot's understanding of its environment:

- **Visual Perception**: Cameras provide rich information about object appearance, color, texture, and spatial relationships. This is essential for object recognition, scene understanding, and human interaction.

- **Depth Perception**: Depth sensors, stereo cameras, or structured light systems provide 3D information about the environment, enabling accurate spatial reasoning and manipulation planning.

- **Inertial Perception**: IMUs provide information about the robot's orientation, acceleration, and angular velocity, which is critical for balance control and motion estimation.

- **Tactile Perception**: Force/torque sensors, tactile sensors, and joint torque feedback provide information about physical interactions with the environment, which is essential for manipulation and safety.

**Temporal Fusion**: Combining sensor information across time to maintain consistent understanding:

- **State Estimation**: Using filtering techniques (Kalman filters, particle filters, etc.) to maintain consistent estimates of robot state and environment state over time.

- **Tracking**: Maintaining consistent tracking of objects, humans, and other dynamic elements in the environment.

- **Prediction**: Using temporal models to predict future states of the environment and plan accordingly.

### Action-Perception Loops

Autonomous humanoid robots operate in continuous action-perception loops where perception guides action and action affects perception:

**Closed-loop Control**: Maintaining continuous feedback between perception and action to ensure robust performance:

- **Reactive Control**: Immediate responses to perceptual inputs for safety and basic functionality.

- **Predictive Control**: Anticipating the effects of actions on perception and planning accordingly.

- **Adaptive Control**: Adjusting control strategies based on perceptual feedback and task success.

**Active Perception**: Controlling sensors and viewpoints to gather the most useful information:

- **Gaze Control**: Directing cameras and attention to relevant parts of the environment.

- **View Planning**: Planning sensor movements to gather information needed for task completion.

- **Information Gain**: Selecting actions that maximize information gain about uncertain aspects of the environment.

## Autonomous Task Planning and Execution

### Hierarchical Task Networks

Autonomous humanoid robots must be able to decompose complex goals into executable sequences of actions. Hierarchical Task Networks (HTNs) provide a framework for organizing complex behaviors into manageable subtasks:

**Task Decomposition**: Breaking high-level goals into sequences of lower-level actions:

- **Goal Analysis**: Understanding the requirements and constraints of high-level goals.

- **Method Selection**: Choosing appropriate methods for achieving subgoals.

- **Constraint Propagation**: Ensuring that subtask solutions satisfy overall goal constraints.

- **Resource Management**: Managing computational and physical resources during task execution.

**Temporal Planning**: Coordinating the timing of actions and managing concurrent activities:

- **Synchronization**: Coordinating multiple subsystems to work together effectively.

- **Resource Allocation**: Managing shared resources like computation, memory, and actuators.

- **Deadline Management**: Ensuring that time-critical actions are completed on schedule.

### Behavior Trees for Autonomous Control

Behavior trees provide a flexible framework for organizing autonomous robot behaviors:

**Tree Structure**: Organizing robot behaviors in a hierarchical tree structure:

- **Composite Nodes**: Control flow nodes that manage the execution of child nodes (sequences, selectors, parallel nodes).

- **Decorator Nodes**: Modify the behavior of child nodes (inverter, repeater, conditional).

- **Leaf Nodes**: Atomic behaviors that perform specific actions or make decisions.

**Dynamic Reconfiguration**: Allowing the behavior tree to adapt during execution:

- **Condition Monitoring**: Continuously monitoring conditions and switching between behaviors as needed.

- **Fallback Mechanisms**: Providing alternative behaviors when primary behaviors fail.

- **Learning Integration**: Incorporating learned behaviors and strategies into the tree structure.

### Learning-Based Planning

Autonomous humanoid robots can improve their planning capabilities through learning:

**Reinforcement Learning**: Learning optimal policies through interaction with the environment:

- **Reward Shaping**: Designing reward functions that encourage desired behaviors.

- **Exploration vs. Exploitation**: Balancing exploration of new strategies with exploitation of known effective strategies.

- **Transfer Learning**: Applying learned policies to new but related tasks.

**Imitation Learning**: Learning from demonstrations by humans or other agents:

- **Behavior Cloning**: Learning to imitate demonstrated behaviors.

- **Inverse Reinforcement Learning**: Learning the underlying reward function from demonstrations.

- **One-shot Learning**: Learning new behaviors from minimal demonstrations.

## Human-Robot Interaction and Social Autonomy

### Natural Interaction Paradigms

Autonomous humanoid robots must interact naturally with humans to be effective in human environments:

**Multimodal Communication**: Using multiple communication channels simultaneously:

- **Verbal Communication**: Understanding and generating natural language.

- **Gestural Communication**: Using and interpreting gestures and body language.

- **Facial Expressions**: Using facial expressions to communicate robot state and intentions.

- **Proxemic Behavior**: Managing personal space and social distance appropriately.

**Social Norm Compliance**: Following social norms and expectations:

- **Turn-taking**: Following conversational turn-taking conventions.

- **Attention Management**: Appropriately managing attention between multiple humans.

- **Social Navigation**: Navigating around humans following social conventions.

- **Politeness**: Exhibiting polite behaviors and respecting human preferences.

### Collaborative Task Execution

Autonomous humanoid robots must be able to work effectively with humans on shared tasks:

**Intent Recognition**: Understanding human intentions and goals:

- **Behavior Prediction**: Predicting human actions based on observed behavior.

- **Goal Inference**: Inferring human goals from observed actions and environmental context.

- **Attention Tracking**: Understanding what the human is attending to and interested in.

**Collaborative Planning**: Coordinating actions with humans:

- **Role Assignment**: Determining appropriate roles for human and robot in collaborative tasks.

- **Action Synchronization**: Coordinating timing of actions between human and robot.

- **Failure Recovery**: Handling failures in collaborative tasks gracefully.

## Safety and Robustness in Autonomous Systems

### Safety Architecture for Autonomous Humanoids

Safety is paramount in autonomous humanoid systems, particularly when operating in human environments:

**Multi-layered Safety**: Implementing safety at multiple levels of the system:

- **Hardware Safety**: Intrinsic safety features in robot hardware and actuators.

- **Low-level Safety**: Real-time safety checks on joint positions, velocities, and torques.

- **Mid-level Safety**: Safety checks on planned motions and trajectories.

- **High-level Safety**: Safety reasoning at the task and behavior level.

**Fail-safe Mechanisms**: Ensuring safe operation even when components fail:

- **Graceful Degradation**: Maintaining safe operation with reduced capabilities when components fail.

- **Emergency Procedures**: Predefined responses to safety-critical situations.

- **Human Override**: Allowing human intervention when autonomous operation is insufficient.

### Robustness to Environmental Variations

Autonomous humanoid robots must operate robustly in diverse and changing environments:

**Environmental Adaptation**: Adjusting behavior based on environmental conditions:

- **Surface Adaptation**: Adjusting walking patterns for different floor surfaces.

- **Lighting Adaptation**: Maintaining perception capabilities under varying lighting conditions.

- **Acoustic Adaptation**: Maintaining communication capabilities under varying acoustic conditions.

**Uncertainty Management**: Operating effectively under uncertainty:

- **Probabilistic Reasoning**: Using probabilistic models to reason under uncertainty.

- **Robust Planning**: Planning that accounts for uncertainty in the environment and robot state.

- **Active Information Gathering**: Taking actions specifically to reduce uncertainty.

## Learning and Adaptation in Autonomous Systems

### Lifelong Learning for Humanoid Robots

Autonomous humanoid robots must continuously learn and adapt throughout their operational lifetime:

**Incremental Learning**: Learning new skills and information without forgetting previous learning:

- **Catastrophic Forgetting Prevention**: Techniques to prevent loss of previously learned information.

- **Skill Transfer**: Applying learned skills to new but related tasks.

- **Context Learning**: Learning to adapt behavior based on context.

**Interactive Learning**: Learning from interaction with humans and the environment:

- **Learning from Demonstration**: Acquiring new skills by observing human demonstrations.

- **Learning from Correction**: Improving performance based on human corrections and feedback.

- **Learning from Natural Interaction**: Learning through natural human-robot interaction.

### Self-Improvement Mechanisms

Autonomous humanoid robots should be able to improve their own performance over time:

**Performance Monitoring**: Continuously monitoring task performance and identifying areas for improvement:

- **Success/Failure Analysis**: Analyzing successful and failed task executions to identify patterns.

- **Efficiency Metrics**: Monitoring task execution efficiency and identifying optimization opportunities.

- **User Satisfaction**: Monitoring human user satisfaction and adapting accordingly.

**Self-Reflection**: Higher-level reasoning about robot capabilities and limitations:

- **Capability Assessment**: Understanding what the robot can and cannot do effectively.

- **Learning Prioritization**: Prioritizing learning based on task importance and capability gaps.

- **Goal Adjustment**: Adjusting goals based on capability assessment and environmental constraints.

## Implementation Architecture for Autonomous Humanoid Systems

### System Architecture Overview

The architecture for autonomous humanoid systems must support the integration of diverse capabilities while maintaining real-time performance and safety:

**Modular Design**: Organizing the system into modular components that can be developed and tested independently:

- **Perception Module**: Handles sensor processing and environment understanding.

- **Planning Module**: Handles task and motion planning.

- **Control Module**: Handles low-level control and execution.

- **Learning Module**: Handles learning and adaptation.

- **Interaction Module**: Handles human-robot interaction.

**Communication Framework**: Enabling effective communication between modules:

- **Message Passing**: Using standardized message formats for inter-module communication.

- **Data Synchronization**: Ensuring consistent data across modules.

- **Timing Coordination**: Coordinating timing between modules with different update rates.

### Real-time Considerations

Autonomous humanoid systems must meet strict real-time requirements:

**Timing Requirements**: Different system components have different timing requirements:

- **Control Loop**: Typically 100-1000 Hz for balance and motion control.

- **Perception Loop**: 10-30 Hz for environment understanding.

- **Planning Loop**: 1-10 Hz for high-level planning.

- **Interaction Loop**: 1-5 Hz for human interaction.

**Resource Management**: Efficiently managing computational resources:

- **Priority Scheduling**: Ensuring critical tasks receive necessary resources.

- **Load Balancing**: Distributing computational load across available resources.

- **Memory Management**: Efficiently managing memory for real-time operation.

## Case Studies in Humanoid Autonomy

### Domestic Service Robot

A domestic service robot represents a complex autonomous system that must operate in human homes:

**Capabilities Required**:
- Navigation through cluttered home environments
- Object recognition and manipulation for household tasks
- Human interaction for communication and instruction
- Task planning for complex household chores
- Safety for operation around family members

**Technical Challenges**:
- Adapting to diverse home layouts and furnishings
- Handling household objects of varying shapes and materials
- Understanding natural language commands in domestic contexts
- Maintaining safety around children and pets
- Learning household routines and preferences

### Industrial Collaborative Robot

A collaborative humanoid robot in industrial settings must work safely with human workers:

**Capabilities Required**:
- Safe physical interaction with humans
- Complex manipulation for assembly tasks
- Real-time response to human actions
- Quality control and inspection capabilities
- Integration with existing industrial systems

**Technical Challenges**:
- Ensuring safety in close human-robot collaboration
- Handling industrial parts with precision requirements
- Adapting to variations in human work patterns
- Maintaining productivity while ensuring safety
- Integrating with factory automation systems

### Healthcare Assistant Robot

Healthcare robots must operate in sensitive environments with vulnerable populations:

**Capabilities Required**:
- Gentle and safe physical interaction
- Understanding of medical terminology and procedures
- Emotional intelligence for patient interaction
- Sterile environment operation
- Emergency response capabilities

**Technical Challenges**:
- Ensuring safety with patients who may have mobility issues
- Understanding complex medical contexts
- Maintaining sterility requirements
- Handling emergency situations appropriately
- Respecting patient privacy and dignity

## Future Directions in Humanoid Autonomy

### Emerging Technologies

Several emerging technologies will shape the future of humanoid autonomy:

**Advanced AI**: Next-generation AI technologies will enable more sophisticated autonomous behaviors:

- **Foundation Models**: Large foundation models for vision, language, and action that can be adapted to robotics tasks.

- **Neuromorphic Computing**: Brain-inspired computing architectures that may enable more efficient autonomous operation.

- **Quantum Computing**: Potential applications in optimization and planning for complex autonomous systems.

**Advanced Sensing**: Next-generation sensors will provide richer environmental information:

- **Event-based Vision**: Cameras that capture changes in the environment rather than full frames.

- **Advanced Tactile Sensing**: More sophisticated tactile sensors for better manipulation.

- **Environmental Sensors**: Advanced sensors for detecting air quality, temperature, and other environmental factors.

### Research Challenges

Several research challenges remain in achieving full humanoid autonomy:

**Generalization**: Developing systems that can generalize across diverse tasks and environments:

- **Transfer Learning**: Effectively transferring knowledge between different tasks and environments.

- **Few-shot Learning**: Learning new tasks from minimal demonstrations or experience.

- **Zero-shot Learning**: Performing tasks without prior experience based on high-level descriptions.

**Scalability**: Scaling autonomous systems to operate effectively with large numbers of robots:

- **Multi-robot Coordination**: Coordinating the behavior of multiple autonomous robots.

- **Cloud Robotics**: Leveraging cloud computing for enhanced autonomous capabilities.

- **Distributed Learning**: Learning across multiple robot platforms to improve all robots.

## Conclusion

Humanoid autonomy represents the culmination of decades of robotics research and development, integrating perception, planning, control, learning, and interaction capabilities into systems that can operate independently in complex human environments. The journey toward true humanoid autonomy has required advances across multiple disciplines, from mechanical engineering and control theory to artificial intelligence and human-robot interaction.

The autonomous humanoid systems of the future will be characterized by their ability to learn continuously, adapt to new situations, interact naturally with humans, and operate safely in diverse environments. These systems will not only execute predefined tasks but will understand context, make decisions under uncertainty, and collaborate effectively with humans as partners rather than tools.

The chapters in this book have provided the foundational knowledge necessary to understand and develop these complex autonomous systems. From the mechanical design of humanoid robots and the ROS 2 communication framework, through simulation and perception systems, to advanced AI integration and voice interaction, each component plays a crucial role in the larger autonomy picture.

As we look to the future, the field of humanoid autonomy will continue to evolve, driven by advances in AI, sensing, actuation, and our understanding of human-robot interaction. The challenges ahead are significant, but the potential benefits for society are immense. Autonomous humanoid robots have the potential to assist in homes, workplaces, healthcare facilities, and disaster response scenarios, augmenting human capabilities and improving quality of life.

The development of truly autonomous humanoid robots requires not just technical excellence but also careful consideration of ethical, social, and safety implications. As these systems become more capable and prevalent, we must ensure that they are developed and deployed in ways that benefit humanity and respect human values.

This concludes our exploration of humanoid robotics and autonomy. The knowledge and techniques presented in this book provide a foundation for developing the next generation of autonomous humanoid systems that will work alongside humans as capable, reliable, and safe partners.

---

## References

[1] Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics*. Springer.

[2] Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT press.

[3] Goodrich, M. A., & Schultz, A. C. (2007). Human-robot interaction: a survey. *Foundations and Trends in Human-Computer Interaction*, 1(3), 203-275.

[4] Khatib, O., Park, H. J., & Park, I. W. (2014). A humanoid manipulation platform with compliant and underactuated hands. *IEEE Robotics & Automation Magazine*, 21(4), 30-38.

[5] Brooks, R. (1991). Intelligence without representation. *Artificial Intelligence*, 47(1-3), 139-159.

## Code Examples

### Example 1: Autonomous Humanoid System Manager
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from enum import Enum
from collections import deque
import json

class SystemState(Enum):
    IDLE = 0
    PERCEPTION = 1
    PLANNING = 2
    EXECUTION = 3
    EMERGENCY = 4
    LEARNING = 5

class AutonomousHumanoidManager(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_manager')

        # System state management
        self.current_state = SystemState.IDLE
        self.previous_state = SystemState.IDLE
        self.state_timestamp = self.get_clock().now()

        # Robot state
        self.robot_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            'joint_positions': {},
            'battery_level': 1.0,
            'task_queue': [],
            'current_task': None,
            'safety_status': 'SAFE',
            'interaction_mode': 'AUTONOMOUS'
        }

        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
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

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.camera_callback,
            10
        )

        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/system_status',
            10
        )

        self.task_pub = self.create_publisher(
            String,
            '/task_commands',
            10
        )

        self.safety_pub = self.create_publisher(
            String,
            '/safety_status',
            10
        )

        # Initialize perception components
        self.perception_buffer = deque(maxlen=100)
        self.navigation_map = {}
        self.object_detections = []

        # Initialize planning components
        self.goal_queue = []
        self.current_plan = []
        self.plan_index = 0

        # Initialize safety parameters
        self.safety_distance = 0.5
        self.emergency_stop = False
        self.collision_detected = False

        # Initialize learning components
        self.experience_buffer = deque(maxlen=1000)
        self.performance_metrics = {}

        # Start main control loop
        self.main_loop_timer = self.create_timer(0.1, self.main_control_loop)  # 10 Hz

        # Start safety monitoring
        self.safety_timer = self.create_timer(0.05, self.safety_monitor)  # 20 Hz

        self.get_logger().info('Autonomous Humanoid Manager initialized')

    def joint_state_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.robot_state['joint_positions'][name] = msg.position[i]

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Check for obstacles in front of robot
        front_scan_start = len(msg.ranges) // 2 - 30
        front_scan_end = len(msg.ranges) // 2 + 30

        min_distance = float('inf')
        for i in range(front_scan_start, front_scan_end):
            if i < len(msg.ranges) and not (msg.ranges[i] <= 0.0 or msg.ranges[i] >= float('inf')):
                if msg.ranges[i] < min_distance:
                    min_distance = msg.ranges[i]

        if min_distance < self.safety_distance:
            self.collision_detected = True
            self.get_logger().warn(f'Collision detected: {min_distance:.2f}m')
        else:
            self.collision_detected = False

    def imu_callback(self, msg):
        """Update IMU data for balance monitoring"""
        # Extract orientation from IMU
        self.robot_state['orientation'] = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Check for dangerous orientations
        roll, pitch, _ = self.quaternion_to_euler(*self.robot_state['orientation'])
        if abs(roll) > 1.0 or abs(pitch) > 1.0:  # 57 degrees
            self.get_logger().warn('Dangerous orientation detected')
            self.emergency_stop = True

    def odom_callback(self, msg):
        """Update odometry information"""
        self.robot_state['position'] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def camera_callback(self, msg):
        """Process camera data for perception"""
        # Add to perception buffer for processing
        self.perception_buffer.append({
            'timestamp': msg.header.stamp,
            'encoding': msg.encoding,
            'height': msg.height,
            'width': msg.width
        })

    def voice_command_callback(self, msg):
        """Process voice commands"""
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        # Add to task queue
        self.robot_state['task_queue'].append(command)

        # Switch to planning state if idle
        if self.current_state == SystemState.IDLE:
            self.set_state(SystemState.PLANNING)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        import math
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def set_state(self, new_state):
        """Change system state with proper transition handling"""
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_timestamp = self.get_clock().now()

        self.get_logger().info(f'System state changed from {self.previous_state.name} to {self.current_state.name}')

    def main_control_loop(self):
        """Main control loop for autonomous operation"""
        current_time = self.get_clock().now()

        # State machine for autonomous operation
        if self.current_state == SystemState.IDLE:
            self.handle_idle_state()
        elif self.current_state == SystemState.PERCEPTION:
            self.handle_perception_state()
        elif self.current_state == SystemState.PLANNING:
            self.handle_planning_state()
        elif self.current_state == SystemState.EXECUTION:
            self.handle_execution_state()
        elif self.current_state == SystemState.EMERGENCY:
            self.handle_emergency_state()
        elif self.current_state == SystemState.LEARNING:
            self.handle_learning_state()

        # Publish system status
        status_msg = String()
        status_msg.data = json.dumps({
            'state': self.current_state.name,
            'timestamp': current_time.nanoseconds,
            'battery': self.robot_state['battery_level'],
            'safety': self.robot_state['safety_status']
        })
        self.status_pub.publish(status_msg)

    def handle_idle_state(self):
        """Handle idle state - waiting for tasks"""
        if self.robot_state['task_queue']:
            self.set_state(SystemState.PLANNING)

    def handle_perception_state(self):
        """Handle perception state - processing sensor data"""
        # Process perception data
        self.process_environment_sensing()

        # Update navigation map
        self.update_navigation_map()

        # Detect objects and people
        self.detect_environment_objects()

        # Transition to planning when perception is complete
        if self.robot_state['task_queue']:
            self.set_state(SystemState.PLANNING)
        else:
            self.set_state(SystemState.IDLE)

    def handle_planning_state(self):
        """Handle planning state - creating action plans"""
        if self.robot_state['task_queue']:
            task = self.robot_state['task_queue'][0]
            plan = self.create_task_plan(task)

            if plan:
                self.current_plan = plan
                self.plan_index = 0
                self.robot_state['current_task'] = task
                self.set_state(SystemState.EXECUTION)
            else:
                # Remove failed task
                self.robot_state['task_queue'].pop(0)
                self.set_state(SystemState.IDLE)
        else:
            self.set_state(SystemState.IDLE)

    def handle_execution_state(self):
        """Handle execution state - executing action plans"""
        if self.current_plan and self.plan_index < len(self.current_plan):
            # Execute current plan step
            plan_step = self.current_plan[self.plan_index]
            success = self.execute_plan_step(plan_step)

            if success:
                self.plan_index += 1

                # Log experience for learning
                self.log_experience(plan_step, 'success')

                # Check if plan is complete
                if self.plan_index >= len(self.current_plan):
                    # Mark task as complete
                    completed_task = self.robot_state['current_task']
                    self.robot_state['current_task'] = None

                    # Remove from queue
                    if self.robot_state['task_queue']:
                        self.robot_state['task_queue'].pop(0)

                    # Publish completion
                    completion_msg = String()
                    completion_msg.data = f'task_completed:{completed_task}'
                    self.task_pub.publish(completion_msg)

                    # Go back to idle
                    self.set_state(SystemState.IDLE)
            else:
                # Plan step failed, handle failure
                self.handle_execution_failure()
        else:
            # No plan to execute, go back to idle
            self.set_state(SystemState.IDLE)

    def handle_emergency_state(self):
        """Handle emergency state - safety critical situations"""
        # Stop all motion
        self.stop_robot()

        # Publish emergency status
        emergency_msg = String()
        emergency_msg.data = 'EMERGENCY_STOP'
        self.safety_pub.publish(emergency_msg)

        # Wait for manual intervention or safe conditions
        if not self.emergency_stop and not self.collision_detected:
            self.set_state(SystemState.IDLE)

    def handle_learning_state(self):
        """Handle learning state - improving performance"""
        # Analyze experience data
        self.analyze_experience_data()

        # Update performance metrics
        self.update_performance_metrics()

        # Adjust parameters based on learning
        self.adjust_learning_parameters()

        # Return to normal operation
        self.set_state(SystemState.IDLE)

    def create_task_plan(self, task):
        """Create a plan for executing a task"""
        # This is a simplified planner - in real implementation, this would be more sophisticated
        if 'navigate' in task:
            return [
                {'action': 'move_to', 'target': self.extract_location(task)},
                {'action': 'look_around', 'duration': 2.0}
            ]
        elif 'grasp' in task:
            return [
                {'action': 'move_to_object', 'object': self.extract_object(task)},
                {'action': 'grasp_object', 'object': self.extract_object(task)},
                {'action': 'return_to_base'}
            ]
        elif 'speak' in task:
            return [
                {'action': 'speak', 'text': self.extract_text(task)}
            ]

        return []

    def extract_location(self, task):
        """Extract location from task command"""
        # Simple extraction - in real implementation, use NLP
        if 'kitchen' in task:
            return 'kitchen'
        elif 'living room' in task:
            return 'living_room'
        elif 'bedroom' in task:
            return 'bedroom'
        else:
            return 'unknown'

    def extract_object(self, task):
        """Extract object from task command"""
        # Simple extraction - in real implementation, use NLP
        if 'cup' in task:
            return 'cup'
        elif 'book' in task:
            return 'book'
        elif 'bottle' in task:
            return 'bottle'
        else:
            return 'unknown'

    def extract_text(self, task):
        """Extract text from speak command"""
        # Simple extraction - in real implementation, use NLP
        if 'speak:' in task:
            return task.split('speak:')[1].strip()
        else:
            return task

    def execute_plan_step(self, step):
        """Execute a single step of a plan"""
        action = step.get('action', '')

        if action == 'move_to':
            location = step.get('target', 'unknown')
            return self.execute_navigation(location)
        elif action == 'move_to_object':
            obj = step.get('object', 'unknown')
            return self.execute_object_navigation(obj)
        elif action == 'grasp_object':
            obj = step.get('object', 'unknown')
            return self.execute_grasp(obj)
        elif action == 'speak':
            text = step.get('text', 'Hello')
            return self.execute_speak(text)
        elif action == 'look_around':
            duration = step.get('duration', 2.0)
            return self.execute_look_around(duration)
        elif action == 'return_to_base':
            return self.execute_return_to_base()

        return False

    def execute_navigation(self, location):
        """Execute navigation to a location"""
        # In real implementation, this would send navigation goals
        self.get_logger().info(f'Navigating to {location}')

        # Publish navigation command
        nav_cmd = String()
        nav_cmd.data = f'navigate_to:{location}'
        self.task_pub.publish(nav_cmd)

        return True

    def execute_object_navigation(self, obj):
        """Execute navigation to an object"""
        self.get_logger().info(f'Navigating to {obj}')

        # Publish navigation command
        nav_cmd = String()
        nav_cmd.data = f'navigate_to_object:{obj}'
        self.task_pub.publish(nav_cmd)

        return True

    def execute_grasp(self, obj):
        """Execute grasp action"""
        self.get_logger().info(f'Grasping {obj}')

        # Publish grasp command
        grasp_cmd = String()
        grasp_cmd.data = f'grasp:{obj}'
        self.task_pub.publish(grasp_cmd)

        return True

    def execute_speak(self, text):
        """Execute speech action"""
        self.get_logger().info(f'Speaking: {text}')

        # Publish speech command
        speak_cmd = String()
        speak_cmd.data = f'speak:{text}'
        self.task_pub.publish(speak_cmd)

        return True

    def execute_look_around(self, duration):
        """Execute look around action"""
        self.get_logger().info(f'Looking around for {duration}s')

        # Publish look command
        look_cmd = String()
        look_cmd.data = f'look_around:{duration}'
        self.task_pub.publish(look_cmd)

        # Simulate duration with timer
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(0.1)

        return True

    def execute_return_to_base(self):
        """Execute return to base action"""
        self.get_logger().info('Returning to base')

        # Publish return command
        return_cmd = String()
        return_cmd.data = 'return_to_base'
        self.task_pub.publish(return_cmd)

        return True

    def process_environment_sensing(self):
        """Process environment sensor data"""
        # This would integrate data from all sensors
        # For now, just log that sensing is happening
        self.get_logger().info('Processing environment sensing')

    def update_navigation_map(self):
        """Update navigation map based on sensor data"""
        # This would update the robot's map of the environment
        # For now, just log that mapping is happening
        self.get_logger().info('Updating navigation map')

    def detect_environment_objects(self):
        """Detect objects in the environment"""
        # This would run object detection algorithms
        # For now, just log that detection is happening
        self.get_logger().info('Detecting environment objects')

    def handle_execution_failure(self):
        """Handle failure during plan execution"""
        self.get_logger().warn('Plan execution failed, handling failure')

        # Log failure for learning
        self.log_experience(self.current_plan[self.plan_index], 'failure')

        # Try alternative approach or go to emergency state
        if self.emergency_stop or self.collision_detected:
            self.set_state(SystemState.EMERGENCY)
        else:
            # Remove failed task and continue
            if self.robot_state['task_queue']:
                self.robot_state['task_queue'].pop(0)
            self.set_state(SystemState.IDLE)

    def safety_monitor(self):
        """Monitor safety conditions"""
        # Check for emergency conditions
        if self.emergency_stop or self.collision_detected:
            if self.current_state != SystemState.EMERGENCY:
                self.set_state(SystemState.EMERGENCY)

    def stop_robot(self):
        """Emergency stop for robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def log_experience(self, action, outcome):
        """Log experience for learning"""
        experience = {
            'action': action,
            'outcome': outcome,
            'timestamp': self.get_clock().now().nanoseconds,
            'robot_state': self.robot_state.copy()
        }
        self.experience_buffer.append(experience)

    def analyze_experience_data(self):
        """Analyze experience data for learning"""
        # Analyze successful and failed actions
        successful_actions = []
        failed_actions = []

        for exp in list(self.experience_buffer)[-50:]:  # Last 50 experiences
            if exp['outcome'] == 'success':
                successful_actions.append(exp)
            else:
                failed_actions.append(exp)

        # Update performance metrics
        total_actions = len(successful_actions) + len(failed_actions)
        if total_actions > 0:
            success_rate = len(successful_actions) / total_actions
            self.performance_metrics['success_rate'] = success_rate
            self.get_logger().info(f'Learning: Success rate = {success_rate:.2f}')

    def update_performance_metrics(self):
        """Update performance metrics"""
        # Update various performance metrics
        self.performance_metrics['battery_consumption_rate'] = 0.01  # Simplified
        self.performance_metrics['task_completion_time_avg'] = 30.0  # Simplified
        self.performance_metrics['navigation_success_rate'] = 0.95  # Simplified

    def adjust_learning_parameters(self):
        """Adjust parameters based on learning"""
        # Adjust parameters based on performance
        if 'success_rate' in self.performance_metrics:
            success_rate = self.performance_metrics['success_rate']
            if success_rate < 0.7:
                # Be more conservative
                self.safety_distance = 0.7  # Increase safety distance
            elif success_rate > 0.9:
                # Be more aggressive
                self.safety_distance = 0.4  # Decrease safety distance

def main(args=None):
    rclpy.init(args=args)
    manager = AutonomousHumanoidManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        manager.get_logger().info('Shutting down autonomous manager')
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Humanoid Autonomy Behavior Tree
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import random
from enum import Enum

class NodeStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2

class BehaviorNode:
    def __init__(self, name):
        self.name = name
        self.children = []
        self.status = NodeStatus.RUNNING

    def add_child(self, child):
        self.children.append(child)

    def tick(self):
        """Execute the behavior and return status"""
        pass

class CompositeNode(BehaviorNode):
    def __init__(self, name):
        super().__init__(name)

class DecoratorNode(BehaviorNode):
    def __init__(self, name, child=None):
        super().__init__(name)
        if child:
            self.add_child(child)

class ActionNode(BehaviorNode):
    def __init__(self, name):
        super().__init__(name)

# Composite nodes
class SequenceNode(CompositeNode):
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status == NodeStatus.FAILURE:
                return NodeStatus.FAILURE
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.SUCCESS

class SelectorNode(CompositeNode):
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status == NodeStatus.SUCCESS:
                return NodeStatus.SUCCESS
            elif status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
        return NodeStatus.FAILURE

class ParallelNode(CompositeNode):
    def __init__(self, name, success_threshold=1, failure_threshold=1):
        super().__init__(name)
        self.success_threshold = success_threshold
        self.failure_threshold = failure_threshold

    def tick(self):
        success_count = 0
        failure_count = 0
        running_count = 0

        for child in self.children:
            status = child.tick()
            if status == NodeStatus.SUCCESS:
                success_count += 1
            elif status == NodeStatus.FAILURE:
                failure_count += 1
            elif status == NodeStatus.RUNNING:
                running_count += 1

        if success_count >= self.success_threshold:
            return NodeStatus.SUCCESS
        elif failure_count >= self.failure_threshold:
            return NodeStatus.FAILURE
        elif running_count > 0:
            return NodeStatus.RUNNING
        else:
            return NodeStatus.FAILURE

# Decorator nodes
class InverterNode(DecoratorNode):
    def tick(self):
        if self.children:
            status = self.children[0].tick()
            if status == NodeStatus.SUCCESS:
                return NodeStatus.FAILURE
            elif status == NodeStatus.FAILURE:
                return NodeStatus.SUCCESS
            else:
                return status
        return NodeStatus.FAILURE

class RepeatNode(DecoratorNode):
    def __init__(self, name, child=None, max_iterations=-1):
        super().__init__(name, child)
        self.max_iterations = max_iterations
        self.current_iteration = 0

    def tick(self):
        if self.children:
            while self.max_iterations == -1 or self.current_iteration < self.max_iterations:
                status = self.children[0].tick()
                if status == NodeStatus.FAILURE:
                    self.current_iteration = 0
                    return NodeStatus.FAILURE
                elif status == NodeStatus.SUCCESS:
                    self.current_iteration += 1
                    if self.max_iterations != -1 and self.current_iteration >= self.max_iterations:
                        self.current_iteration = 0
                        return NodeStatus.SUCCESS
                else:
                    return NodeStatus.RUNNING
        return NodeStatus.FAILURE

# Action nodes for humanoid robot
class MoveToLocationAction(ActionNode):
    def __init__(self, name, location):
        super().__init__(name)
        self.location = location
        self.progress = 0

    def tick(self):
        # Simulate movement progress
        self.progress += 0.1
        if self.progress >= 1.0:
            self.progress = 0
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

class CheckObstaclesAction(ActionNode):
    def __init__(self, name):
        super().__init__(name)
        self.obstacle_detected = False

    def tick(self):
        # Simulate obstacle detection (random for demo)
        self.obstacle_detected = random.random() < 0.1  # 10% chance
        if self.obstacle_detected:
            return NodeStatus.FAILURE
        return NodeStatus.SUCCESS

class GraspObjectAction(ActionNode):
    def __init__(self, name, object_name):
        super().__init__(name)
        self.object_name = object_name
        self.grasp_success = False

    def tick(self):
        # Simulate grasp attempt
        if random.random() < 0.8:  # 80% success rate
            self.grasp_success = True
            return NodeStatus.SUCCESS
        else:
            self.grasp_success = False
            return NodeStatus.FAILURE

class SpeakAction(ActionNode):
    def __init__(self, name, text):
        super().__init__(name)
        self.text = text

    def tick(self):
        # Simulate speaking
        print(f"Robot says: {self.text}")
        return NodeStatus.SUCCESS

class AutonomousHumanoidBehaviorTree(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_bt')

        # Create the behavior tree structure
        self.root = self.create_behavior_tree()

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/behavior_status', 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )

        # Timer for running the behavior tree
        self.bt_timer = self.create_timer(0.1, self.run_behavior_tree)  # 10 Hz

        self.get_logger().info('Autonomous Humanoid Behavior Tree initialized')

    def create_behavior_tree(self):
        """Create the behavior tree structure"""
        # Main sequence: check safety, then execute task
        main_sequence = SequenceNode("MainSequence")

        # Safety check parallel node
        safety_check = ParallelNode("SafetyCheck", success_threshold=2, failure_threshold=1)
        safety_check.add_child(CheckObstaclesAction("CheckFrontObstacle"))
        safety_check.add_child(CheckObstaclesAction("CheckSideObstacle"))
        safety_check.add_child(CheckObstaclesAction("CheckRearObstacle"))

        # Task execution selector
        task_selector = SelectorNode("TaskSelector")

        # Navigation task
        nav_sequence = SequenceNode("NavigationTask")
        nav_sequence.add_child(MoveToLocationAction("MoveToKitchen", "kitchen"))
        nav_sequence.add_child(SpeakAction("AnnounceArrival", "I have arrived at the kitchen."))

        # Manipulation task
        manipulation_sequence = SequenceNode("ManipulationTask")
        manipulation_sequence.add_child(MoveToLocationAction("MoveToTable", "table"))
        manipulation_sequence.add_child(GraspObjectAction("GraspCup", "cup"))
        manipulation_sequence.add_child(SpeakAction("ConfirmGrasp", "I have grasped the cup successfully."))

        # Add tasks to selector
        task_selector.add_child(nav_sequence)
        task_selector.add_child(manipulation_sequence)

        # Build main tree
        main_sequence.add_child(safety_check)
        main_sequence.add_child(task_selector)

        return main_sequence

    def laser_callback(self, msg):
        """Handle laser scan data"""
        # Update obstacle detection in behavior tree
        pass

    def run_behavior_tree(self):
        """Run the behavior tree"""
        status = self.root.tick()

        # Publish status
        status_msg = String()
        status_msg.data = f"BT_Status: {status.name}"
        self.status_pub.publish(status_msg)

        # Log status
        self.get_logger().info(f'Behavior tree status: {status.name}')

    def add_task_to_tree(self, task_type, **kwargs):
        """Dynamically add tasks to the behavior tree"""
        if task_type == "navigation":
            nav_action = MoveToLocationAction(f"MoveTo{kwargs['location']}", kwargs['location'])
            # Add to appropriate part of tree
            pass
        elif task_type == "manipulation":
            grasp_action = GraspObjectAction(f"Grasp{kwargs['object']}", kwargs['object'])
            # Add to appropriate part of tree
            pass

def main(args=None):
    rclpy.init(args=args)
    bt_node = AutonomousHumanoidBehaviorTree()

    try:
        rclpy.spin(bt_node)
    except KeyboardInterrupt:
        bt_node.get_logger().info('Shutting down behavior tree')
    finally:
        bt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Humanoid Learning and Adaptation System
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import pickle
import os
from collections import deque
import time
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
import joblib

class HumanoidLearningSystem(Node):
    def __init__(self):
        super().__init__('humanoid_learning_system')

        # Initialize learning components
        self.experience_buffer = deque(maxlen=10000)  # Store experiences
        self.performance_history = deque(maxlen=1000)  # Track performance
        self.skill_models = {}  # Store learned skill models
        self.adaptation_parameters = {}  # Store adaptation parameters

        # Initialize default parameters
        self.initialize_parameters()

        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.task_feedback_sub = self.create_subscription(
            String,
            '/task_feedback',
            self.task_feedback_callback,
            10
        )

        self.performance_sub = self.create_subscription(
            Float32,
            '/performance_metric',
            self.performance_callback,
            10
        )

        # Create publishers
        self.adaptation_pub = self.create_publisher(
            String,
            '/adaptation_commands',
            10
        )

        self.learning_status_pub = self.create_publisher(
            String,
            '/learning_status',
            10
        )

        # Start learning timer
        self.learning_timer = self.create_timer(1.0, self.learning_cycle)  # 1 Hz

        # Start adaptation timer
        self.adaptation_timer = self.create_timer(0.5, self.adaptation_cycle)  # 2 Hz

        self.get_logger().info('Humanoid Learning System initialized')

    def initialize_parameters(self):
        """Initialize learning parameters"""
        # Default adaptation parameters
        self.adaptation_parameters = {
            'learning_rate': 0.1,
            'exploration_rate': 0.3,
            'performance_threshold': 0.8,
            'adaptation_window': 50,
            'min_experiences': 10,
            'model_update_frequency': 100
        }

        # Performance metrics
        self.current_performance = 0.0
        self.performance_trend = 0.0
        self.experience_count = 0

    def joint_state_callback(self, msg):
        """Record joint state as part of experience"""
        state_vector = list(msg.position) + list(msg.velocity)
        self.current_state = np.array(state_vector)

    def task_feedback_callback(self, msg):
        """Handle task feedback for learning"""
        feedback_data = msg.data.split(':')
        if len(feedback_data) >= 3:
            task_name = feedback_data[0]
            outcome = feedback_data[1]  # 'success' or 'failure'
            metrics_str = feedback_data[2]

            # Parse metrics
            try:
                metrics = eval(metrics_str)  # In production, use safer parsing
            except:
                metrics = {}

            # Create experience tuple
            experience = {
                'task': task_name,
                'outcome': outcome,
                'metrics': metrics,
                'timestamp': time.time(),
                'state': getattr(self, 'current_state', np.array([]))
            }

            # Add to experience buffer
            self.experience_buffer.append(experience)
            self.experience_count += 1

            self.get_logger().info(f'Logged experience: {task_name}, {outcome}')

    def performance_callback(self, msg):
        """Update current performance metric"""
        old_performance = self.current_performance
        self.current_performance = msg.data

        # Calculate trend
        if old_performance != 0:
            self.performance_trend = (self.current_performance - old_performance) / old_performance

        # Add to performance history
        self.performance_history.append(self.current_performance)

    def learning_cycle(self):
        """Main learning cycle"""
        # Check if we have enough experiences to learn
        if len(self.experience_buffer) < self.adaptation_parameters['min_experiences']:
            return

        # Analyze recent experiences
        recent_experiences = list(self.experience_buffer)[-self.adaptation_parameters['adaptation_window']:]

        # Calculate success rate
        successful_tasks = sum(1 for exp in recent_experiences if exp['outcome'] == 'success')
        success_rate = successful_tasks / len(recent_experiences) if recent_experiences else 0

        # Update performance metric
        self.current_performance = success_rate

        # Check if we should update models
        if self.experience_count % self.adaptation_parameters['model_update_frequency'] == 0:
            self.update_skill_models()

        # Publish learning status
        status_msg = String()
        status_msg.data = f"Learning Status - Success Rate: {success_rate:.3f}, Experiences: {len(self.experience_buffer)}"
        self.learning_status_pub.publish(status_msg)

    def update_skill_models(self):
        """Update learned skill models"""
        self.get_logger().info('Updating skill models...')

        # Group experiences by task type
        task_groups = {}
        for exp in self.experience_buffer:
            task = exp['task']
            if task not in task_groups:
                task_groups[task] = []
            task_groups[task].append(exp)

        # Train model for each task type
        for task, experiences in task_groups.items():
            if len(experiences) >= self.adaptation_parameters['min_experiences']:
                self.train_task_model(task, experiences)

    def train_task_model(self, task_name, experiences):
        """Train a model for a specific task"""
        # Prepare training data
        X = []  # Features (environment state, parameters)
        y = []  # Targets (performance metrics, success/failure)

        for exp in experiences:
            # Extract features from experience
            state_features = exp['state'] if len(exp['state']) > 0 else np.array([0])
            param_features = self.extract_task_parameters(exp['metrics'])

            # Combine features
            features = np.concatenate([state_features, param_features])

            # Extract target (success = 1, failure = 0, or continuous performance metric)
            if exp['outcome'] == 'success':
                target = 1.0
            elif exp['outcome'] == 'failure':
                target = 0.0
            else:
                # Use performance metric if available
                target = exp['metrics'].get('performance', 0.5)

            X.append(features)
            y.append(target)

        if len(X) > 0:
            # Convert to numpy arrays
            X = np.array(X)
            y = np.array(y)

            # Train model (using Random Forest for this example)
            try:
                model = RandomForestRegressor(n_estimators=100, random_state=42)
                model.fit(X, y)

                # Store the model
                self.skill_models[task_name] = model

                # Save model to file
                model_path = f"/tmp/{task_name}_model.pkl"
                joblib.dump(model, model_path)

                self.get_logger().info(f'Trained model for task: {task_name}, samples: {len(X)}')

            except Exception as e:
                self.get_logger().error(f'Error training model for {task_name}: {e}')

    def extract_task_parameters(self, metrics):
        """Extract relevant parameters from task metrics"""
        # Extract parameters that might affect task success
        params = []

        # Time-based parameters
        params.append(metrics.get('execution_time', 0))
        params.append(metrics.get('planning_time', 0))

        # Distance/position parameters
        params.append(metrics.get('distance_traveled', 0))
        params.append(metrics.get('final_position_error', 0))

        # Energy consumption
        params.append(metrics.get('energy_consumed', 0))

        # Joint-related parameters
        params.append(metrics.get('max_joint_velocity', 0))
        params.append(metrics.get('avg_joint_torque', 0))

        return np.array(params)

    def adaptation_cycle(self):
        """Adaptation cycle - adjust parameters based on learning"""
        if not self.performance_history:
            return

        # Calculate recent performance
        recent_performance = np.mean(list(self.performance_history)[-10:]) if len(self.performance_history) >= 10 else np.mean(self.performance_history)

        # Determine adaptation strategy
        adaptation_commands = []

        if recent_performance < self.adaptation_parameters['performance_threshold']:
            # Performance is low, increase exploration
            self.adaptation_parameters['exploration_rate'] = min(0.8, self.adaptation_parameters['exploration_rate'] + 0.05)
            adaptation_commands.append('increase_exploration')

            # Be more conservative with movements
            adaptation_commands.append('reduce_movement_speed')
            adaptation_commands.append('increase_safety_margin')

        else:
            # Performance is good, decrease exploration
            self.adaptation_parameters['exploration_rate'] = max(0.1, self.adaptation_parameters['exploration_rate'] - 0.01)

            if self.performance_trend > 0:
                # Performance is improving, be more confident
                adaptation_commands.append('increase_movement_speed')
                adaptation_commands.append('reduce_planning_caution')

        # Publish adaptation commands
        if adaptation_commands:
            cmd_msg = String()
            cmd_msg.data = ','.join(adaptation_commands)
            self.adaptation_pub.publish(cmd_msg)

            self.get_logger().info(f'Adaptation commands: {cmd_msg.data}')

    def predict_task_success(self, task_name, state, parameters):
        """Predict the success of a task given current state and parameters"""
        if task_name in self.skill_models:
            model = self.skill_models[task_name]

            # Prepare input features
            features = np.concatenate([state, parameters])
            features = features.reshape(1, -1)

            # Make prediction
            prediction = model.predict(features)[0]

            return prediction
        else:
            # No model available, return default success probability
            return 0.5

    def save_learning_state(self, filepath):
        """Save the current learning state to file"""
        learning_state = {
            'experience_buffer': list(self.experience_buffer),
            'performance_history': list(self.performance_history),
            'skill_models': self.skill_models,
            'adaptation_parameters': self.adaptation_parameters,
            'current_performance': self.current_performance,
            'performance_trend': self.performance_trend,
            'experience_count': self.experience_count
        }

        with open(filepath, 'wb') as f:
            pickle.dump(learning_state, f)

        self.get_logger().info(f'Learning state saved to {filepath}')

    def load_learning_state(self, filepath):
        """Load learning state from file"""
        if os.path.exists(filepath):
            with open(filepath, 'rb') as f:
                learning_state = pickle.load(f)

            self.experience_buffer = deque(learning_state['experience_buffer'], maxlen=10000)
            self.performance_history = deque(learning_state['performance_history'], maxlen=1000)
            self.skill_models = learning_state['skill_models']
            self.adaptation_parameters = learning_state['adaptation_parameters']
            self.current_performance = learning_state['current_performance']
            self.performance_trend = learning_state['performance_trend']
            self.experience_count = learning_state['experience_count']

            self.get_logger().info(f'Learning state loaded from {filepath}')
        else:
            self.get_logger().warn(f'Learning state file not found: {filepath}')

def main(args=None):
    rclpy.init(args=args)
    learning_system = HumanoidLearningSystem()

    try:
        rclpy.spin(learning_system)
    except KeyboardInterrupt:
        # Save learning state before shutdown
        learning_system.save_learning_state('/tmp/humanoid_learning_state.pkl')
        learning_system.get_logger().info('Shutting down learning system')
    finally:
        learning_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 4: Humanoid Autonomy Launch File
```xml
<launch>
  <!-- Robot configuration -->
  <arg name="robot_namespace" default="/humanoid_robot"/>
  <arg name="use_sim_time" default="false"/>
  <arg name="model" default="humanoid_32dof"/>

  <!-- Autonomous humanoid system -->
  <group>
    <push-ros-namespace namespace="$(var robot_namespace)"/>

    <!-- Robot state publisher -->
    <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
      <param name="robot_description" value="$(var robot_description)"/>
    </node>

    <!-- Joint state publisher -->
    <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
      <param name="rate" value="50"/>
    </node>

    <!-- Sensor processing -->
    <node pkg="imu_filter_madgwick" exec="imu_filter_node" name="imu_filter" output="screen">
      <param name="use_magnetic_field_msg" value="false"/>
      <param name="publish_tf" value="false"/>
      <param name="world_frame" value="enu"/>
    </node>

    <!-- Perception system -->
    <node pkg="pointcloud_to_laserscan" exec="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan" output="screen">
      <param name="target_frame" value="base_link"/>
      <param name="source_frame" value="lidar_link"/>
      <param name="min_height" value="-0.2"/>
      <param name="max_height" value="0.8"/>
      <param name="range_min" value="0.1"/>
      <param name="range_max" value="20.0"/>
    </node>

    <!-- Navigation system -->
    <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
      <arg name="use_sim_time" value="$(var use_sim_time)"/>
    </include>

    <!-- Manipulation system -->
    <node pkg="moveit_ros_move_group" exec="move_group" name="move_group" output="screen">
      <param name="publish_planning_scene" value="true"/>
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="capabilities" value="move_group/MoveGroupCartesianPathService move_group/MoveGroupExecuteTrajectoryAction"/>
    </node>

    <!-- Voice interaction system -->
    <node pkg="humanoid_voice_control" exec="whisper_robot_commander" name="voice_commander" output="screen">
      <param name="whisper_model" value="base"/>
      <param name="language" value="en"/>
    </node>

    <node pkg="humanoid_voice_control" exec="llm_task_planner" name="llm_planner" output="screen">
      <param name="model" value="gpt-3.5-turbo"/>
      <param name="temperature" value="0.1"/>
    </node>

    <!-- Autonomous system manager -->
    <node pkg="humanoid_autonomy" exec="autonomous_humanoid_manager" name="autonomy_manager" output="screen">
      <param name="safety_distance" value="0.5"/>
      <param name="control_frequency" value="100"/>
      <param name="perception_frequency" value="10"/>
      <param name="planning_frequency" value="1"/>
    </node>

    <!-- Behavior tree system -->
    <node pkg="humanoid_autonomy" exec="autonomous_humanoid_bt" name="behavior_tree" output="screen">
      <param name="tree_update_rate" value="10"/>
    </node>

    <!-- Learning system -->
    <node pkg="humanoid_learning" exec="humanoid_learning_system" name="learning_system" output="screen">
      <param name="learning_rate" value="0.1"/>
      <param name="exploration_rate" value="0.3"/>
    </node>

    <!-- Safety system -->
    <node pkg="humanoid_safety" exec="safety_filter" name="safety_filter" output="screen">
      <param name="emergency_stop_distance" value="0.3"/>
      <param name="max_joint_velocity" value="2.0"/>
      <param name="max_joint_torque" value="100.0"/>
    </node>

    <!-- Human-robot interaction -->
    <node pkg="humanoid_interaction" exec="interaction_manager" name="interaction_manager" output="screen">
      <param name="interaction_modes" value="['autonomous', 'supervised', 'teleoperated']"/>
      <param name="social_navigation" value="true"/>
    </node>

    <!-- System monitor -->
    <node pkg="humanoid_monitoring" exec="system_monitor" name="system_monitor" output="screen">
      <param name="monitor_frequency" value="1"/>
      <param name="cpu_threshold" value="80.0"/>
      <param name="memory_threshold" value="85.0"/>
      <param name="battery_threshold" value="20.0"/>
    </node>

    <!-- Visualization -->
    <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share humanoid_description)/rviz/autonomous_humanoid.rviz"/>

  </group>

  <!-- Gazebo simulation (if needed) -->
  <group if="$(var use_sim_time)">
    <include file="$(find-pkg-share gazebo_ros)/launch/gzserver.launch.py">
      <arg name="world" value="$(find-pkg-share humanoid_gazebo)/worlds/autonomous_humanoid.world"/>
    </include>

    <include file="$(find-pkg-share gazebo_ros)/launch/gzclient.launch.py"/>

    <node pkg="gazebo_ros" exec="spawn_entity.py" output="screen"
          args="-entity humanoid_robot -topic robot_description -x 0 -y 0 -z 1.0"/>
  </group>
</launch>
```

### Example 5: Humanoid Autonomy Performance Monitoring
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import time
import threading
from collections import deque, defaultdict
import statistics
import json

class AutonomyPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('autonomy_performance_monitor')

        # Performance metrics storage
        self.metrics = defaultdict(deque)
        self.metrics['timestamp'] = deque(maxlen=1000)
        self.metrics['cpu_usage'] = deque(maxlen=1000)
        self.metrics['memory_usage'] = deque(maxlen=1000)
        self.metrics['task_success_rate'] = deque(maxlen=1000)
        self.metrics['navigation_success_rate'] = deque(maxlen=1000)
        self.metrics['execution_time'] = deque(maxlen=1000)
        self.metrics['battery_level'] = deque(maxlen=1000)

        # Performance thresholds
        self.thresholds = {
            'cpu_usage': 80.0,
            'memory_usage': 85.0,
            'task_success_rate': 0.7,
            'navigation_success_rate': 0.8,
            'battery_level': 20.0,
            'execution_time': 30.0  # seconds
        }

        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.task_status_sub = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        self.battery_sub = self.create_subscription(
            Float32,
            '/battery_level',
            self.battery_callback,
            10
        )

        # Create publishers
        self.performance_pub = self.create_publisher(
            String,
            '/performance_metrics',
            10
        )

        self.alert_pub = self.create_publisher(
            String,
            '/performance_alerts',
            10
        )

        self.cpu_usage_pub = self.create_publisher(
            Float32,
            '/system/cpu_usage',
            10
        )

        self.memory_usage_pub = self.create_publisher(
            Float32,
            '/system/memory_usage',
            10
        )

        # Start monitoring timers
        self.monitoring_timer = self.create_timer(1.0, self.performance_monitoring_cycle)
        self.system_resources_timer = self.create_timer(5.0, self.system_resources_monitoring)

        # Initialize performance tracking
        self.start_time = time.time()
        self.task_count = 0
        self.successful_tasks = 0
        self.navigation_count = 0
        self.successful_navigations = 0
        self.current_battery = 100.0

        self.get_logger().info('Autonomy Performance Monitor initialized')

    def joint_state_callback(self, msg):
        """Monitor joint state for performance metrics"""
        # Calculate joint velocity metrics
        if msg.velocity:
            avg_velocity = sum(abs(v) for v in msg.velocity) / len(msg.velocity)
            self.metrics['avg_joint_velocity'].append(avg_velocity)

    def odom_callback(self, msg):
        """Monitor odometry for navigation performance"""
        # Track navigation metrics
        pass

    def cmd_vel_callback(self, msg):
        """Monitor commanded velocities"""
        linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        angular_speed = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5

        self.metrics['linear_speed'].append(linear_speed)
        self.metrics['angular_speed'].append(angular_speed)

    def task_status_callback(self, msg):
        """Monitor task execution status"""
        status_data = msg.data.split(':')
        if len(status_data) >= 2:
            task_type = status_data[0]
            status = status_data[1]

            if task_type == 'task':
                self.task_count += 1
                if status == 'success':
                    self.successful_tasks += 1
            elif task_type == 'navigation':
                self.navigation_count += 1
                if status == 'success':
                    self.successful_navigations += 1

        # Calculate success rates
        if self.task_count > 0:
            task_success_rate = self.successful_tasks / self.task_count
            self.metrics['task_success_rate'].append(task_success_rate)

        if self.navigation_count > 0:
            nav_success_rate = self.successful_navigations / self.navigation_count
            self.metrics['navigation_success_rate'].append(nav_success_rate)

    def battery_callback(self, msg):
        """Monitor battery level"""
        self.current_battery = msg.data
        self.metrics['battery_level'].append(self.current_battery)

    def system_resources_monitoring(self):
        """Monitor system resources (CPU, memory)"""
        import psutil

        # Get system CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        self.metrics['cpu_usage'].append(cpu_percent)

        # Get system memory usage
        memory_percent = psutil.virtual_memory().percent
        self.metrics['memory_usage'].append(memory_percent)

        # Publish system metrics
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_usage_pub.publish(cpu_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_usage_pub.publish(memory_msg)

    def performance_monitoring_cycle(self):
        """Main performance monitoring cycle"""
        current_time = time.time()
        self.metrics['timestamp'].append(current_time)

        # Calculate performance metrics
        metrics_report = {
            'timestamp': current_time,
            'uptime': current_time - self.start_time,
            'task_success_rate': self.get_latest_metric('task_success_rate'),
            'navigation_success_rate': self.get_latest_metric('navigation_success_rate'),
            'avg_cpu_usage': self.get_average_metric('cpu_usage'),
            'avg_memory_usage': self.get_average_metric('memory_usage'),
            'current_battery': self.current_battery,
            'total_tasks': self.task_count,
            'successful_tasks': self.successful_tasks,
            'total_navigations': self.navigation_count,
            'successful_navigations': self.successful_navigations
        }

        # Check for performance issues
        alerts = self.check_performance_thresholds(metrics_report)

        # Publish metrics
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics_report)
        self.performance_pub.publish(metrics_msg)

        # Publish alerts if any
        if alerts:
            alert_msg = String()
            alert_msg.data = json.dumps(alerts)
            self.alert_pub.publish(alert_msg)

            for alert in alerts:
                self.get_logger().warn(f'Performance Alert: {alert}')

        # Log summary
        self.log_performance_summary(metrics_report)

    def get_latest_metric(self, metric_name):
        """Get the latest value of a metric"""
        if self.metrics[metric_name]:
            return self.metrics[metric_name][-1]
        return 0.0

    def get_average_metric(self, metric_name):
        """Get the average value of a metric"""
        if self.metrics[metric_name]:
            return sum(self.metrics[metric_name]) / len(self.metrics[metric_name])
        return 0.0

    def check_performance_thresholds(self, metrics):
        """Check if any metrics exceed thresholds"""
        alerts = []

        for metric_name, threshold in self.thresholds.items():
            if metric_name in metrics:
                value = metrics[metric_name]
                if metric_name in ['task_success_rate', 'navigation_success_rate']:
                    # For success rates, alert if below threshold
                    if value < threshold:
                        alerts.append(f'{metric_name} below threshold: {value:.3f} < {threshold}')
                else:
                    # For other metrics, alert if above threshold
                    if value > threshold:
                        alerts.append(f'{metric_name} above threshold: {value:.3f} > {threshold}')

        return alerts

    def log_performance_summary(self, metrics):
        """Log a summary of performance metrics"""
        summary = (
            f"Performance Summary:\n"
            f"  Task Success Rate: {metrics.get('task_success_rate', 0):.3f}\n"
            f"  Navigation Success Rate: {metrics.get('navigation_success_rate', 0):.3f}\n"
            f"  Avg CPU Usage: {metrics.get('avg_cpu_usage', 0):.1f}%\n"
            f"  Avg Memory Usage: {metrics.get('avg_memory_usage', 0):.1f}%\n"
            f"  Battery Level: {metrics.get('current_battery', 0):.1f}%\n"
            f"  Total Tasks: {metrics.get('total_tasks', 0)}\n"
            f"  Successful Tasks: {metrics.get('successful_tasks', 0)}\n"
            f"  Total Navigations: {metrics.get('total_navigations', 0)}\n"
            f"  Successful Navigations: {metrics.get('successful_navigations', 0)}"
        )

        self.get_logger().info(summary)

    def get_performance_report(self):
        """Get a comprehensive performance report"""
        report = {
            'system_health': self.calculate_system_health(),
            'performance_trends': self.calculate_performance_trends(),
            'resource_utilization': self.calculate_resource_utilization(),
            'task_performance': self.calculate_task_performance(),
            'recommendations': self.generate_recommendations()
        }
        return report

    def calculate_system_health(self):
        """Calculate overall system health score"""
        health_score = 100.0

        # Deduct points for various issues
        if self.get_average_metric('cpu_usage') > 80:
            health_score -= 20
        if self.get_average_metric('memory_usage') > 85:
            health_score -= 20
        if self.get_latest_metric('task_success_rate') < 0.7:
            health_score -= 30
        if self.current_battery < 20:
            health_score -= 15

        return max(0, health_score)

    def calculate_performance_trends(self):
        """Calculate performance trends"""
        # Calculate trends over the last N samples
        recent_samples = 10
        recent_task_rates = list(self.metrics['task_success_rate'])[-recent_samples:]
        recent_nav_rates = list(self.metrics['navigation_success_rate'])[-recent_samples:]

        if len(recent_task_rates) >= 2:
            task_trend = (recent_task_rates[-1] - recent_task_rates[0]) / len(recent_task_rates)
        else:
            task_trend = 0

        if len(recent_nav_rates) >= 2:
            nav_trend = (recent_nav_rates[-1] - recent_nav_rates[0]) / len(recent_nav_rates)
        else:
            nav_trend = 0

        return {
            'task_success_trend': task_trend,
            'navigation_success_trend': nav_trend
        }

    def calculate_resource_utilization(self):
        """Calculate resource utilization metrics"""
        return {
            'cpu_avg': self.get_average_metric('cpu_usage'),
            'cpu_peak': max(self.metrics['cpu_usage']) if self.metrics['cpu_usage'] else 0,
            'memory_avg': self.get_average_metric('memory_usage'),
            'memory_peak': max(self.metrics['memory_usage']) if self.metrics['memory_usage'] else 0
        }

    def calculate_task_performance(self):
        """Calculate task performance metrics"""
        return {
            'overall_success_rate': self.successful_tasks / self.task_count if self.task_count > 0 else 0,
            'navigation_success_rate': self.successful_navigations / self.navigation_count if self.navigation_count > 0 else 0,
            'tasks_per_hour': self.task_count / ((time.time() - self.start_time) / 3600) if self.start_time < time.time() else 0
        }

    def generate_recommendations(self):
        """Generate performance improvement recommendations"""
        recommendations = []

        if self.get_average_metric('cpu_usage') > 70:
            recommendations.append("Consider optimizing CPU-intensive processes or upgrading hardware")

        if self.get_average_metric('memory_usage') > 80:
            recommendations.append("Monitor memory usage and consider memory optimization")

        if self.get_latest_metric('task_success_rate') < 0.8:
            recommendations.append("Investigate causes of task failures and improve robustness")

        if self.current_battery < 30:
            recommendations.append("Battery level low - consider returning to charging station")

        return recommendations

def main(args=None):
    rclpy.init(args=args)
    monitor = AutonomyPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Shutting down performance monitor')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```