---
title: "Chapter 7: Whisper, LLM/VLA Planning"
sidebar_position: 1
---

# Chapter 7: Whisper, LLM/VLA Planning

## Introduction to Voice Recognition and Natural Language Processing for Humanoid Robots

The integration of voice recognition and natural language processing represents a transformative capability for humanoid robots, enabling intuitive human-robot interaction through spoken language. Whisper, OpenAI's automatic speech recognition (ASR) system, combined with Large Language Models (LLMs) and Vision-Language-Action (VLA) systems, creates a powerful framework for natural human-robot communication and task execution. This technology stack enables humanoid robots to understand spoken commands, reason about their meaning, and execute complex tasks in response.

Voice-based interaction is particularly valuable for humanoid robots because it leverages the most natural form of human communication. Unlike other input modalities, voice interaction allows for hands-free operation, making it ideal for scenarios where humans are occupied with other tasks or when the robot is operating at a distance from the human operator. The combination of Whisper for speech recognition, LLMs for language understanding and reasoning, and VLA systems for action planning creates a comprehensive pipeline for natural language interaction.

The challenge in implementing voice-based control for humanoid robots lies in the complex pipeline required to convert spoken language into executable robotic actions. This involves multiple stages of processing: converting speech to text, understanding the semantic meaning of commands, mapping language concepts to robot capabilities, and generating appropriate motor actions. Each stage introduces potential sources of error, making robust implementation critical for reliable robot operation.

### The Voice-to-Action Pipeline

The complete voice-to-action pipeline for humanoid robots consists of several interconnected stages:

**Speech Recognition**: Converting spoken language to text using Whisper:
- **Audio Processing**: Capturing and preprocessing audio input
- **Speech-to-Text**: Converting audio to textual representation
- **Real-time Processing**: Handling streaming audio for real-time interaction
- **Noise Robustness**: Filtering background noise and interference

**Language Understanding**: Processing text commands with LLMs:
- **Intent Recognition**: Understanding the user's intention
- **Entity Extraction**: Identifying objects, locations, and parameters
- **Context Awareness**: Maintaining conversation context
- **Ambiguity Resolution**: Handling unclear or ambiguous commands

**Action Planning**: Converting language commands to robot actions:
- **Task Decomposition**: Breaking complex commands into subtasks
- **Capability Mapping**: Matching commands to robot capabilities
- **Motion Planning**: Generating specific motor commands
- **Safety Verification**: Ensuring planned actions are safe

**Execution and Feedback**: Executing actions and providing feedback:
- **Action Execution**: Carrying out planned robotic actions
- **Progress Monitoring**: Tracking task execution
- **Failure Handling**: Managing execution failures
- **User Feedback**: Communicating execution status

### Challenges in Voice-Based Robot Control

Several challenges must be addressed when implementing voice-based control for humanoid robots:

**Real-time Constraints**: Voice processing must maintain real-time performance to enable natural interaction:
- **Processing Latency**: Minimizing delay between speech and action
- **Computational Efficiency**: Optimizing algorithms for real-time execution
- **Memory Management**: Managing memory usage during continuous operation
- **System Responsiveness**: Maintaining system responsiveness during processing

**Robustness**: The system must handle various environmental conditions:
- **Acoustic Conditions**: Different rooms, background noise, reverberation
- **Speaker Variations**: Different accents, speaking rates, vocal characteristics
- **Language Variations**: Different command phrasings for the same action
- **Environmental Changes**: Changing robot and environment states

## Whisper for Automatic Speech Recognition

### Whisper Architecture and Capabilities

Whisper is OpenAI's state-of-the-art automatic speech recognition system that uses a large-scale transformer architecture to convert speech to text. The system was trained on 680,000 hours of multilingual and multitask supervised data, making it robust across different languages, accents, and acoustic conditions. For humanoid robotics applications, Whisper provides reliable speech-to-text conversion that can be integrated into the robot's perception and control pipeline.

Whisper's architecture is built on the transformer model, which allows it to handle variable-length sequences and capture long-range dependencies in audio data. The system processes audio in 30-second chunks and can perform both speech recognition and speech translation, making it suitable for multilingual human-robot interaction scenarios.

**Key Features of Whisper**:
- **Multilingual Support**: Supports multiple languages with a single model
- **Robustness**: Performs well in noisy environments and with various accents
- **Zero-shot Capability**: Can recognize speech without language-specific training
- **Timestamping**: Provides word-level and segment-level timing information
- **Punctuation**: Automatically adds punctuation to transcribed text

### Whisper Integration in Robotics Systems

Integrating Whisper into robotics systems requires careful consideration of real-time performance and resource constraints:

**Real-time Processing**: Implementing streaming audio processing:
- **Audio Buffering**: Managing audio data for continuous processing
- **Chunk Processing**: Processing audio in small chunks for reduced latency
- **Overlap Handling**: Managing chunk boundaries to maintain accuracy
- **VAD Integration**: Using voice activity detection to identify speech segments

**Resource Management**: Optimizing Whisper for robotic deployment:
- **Model Quantization**: Reducing model size for edge deployment
- **GPU Acceleration**: Leveraging GPU computation for faster processing
- **CPU Optimization**: Optimizing for CPU-only systems when needed
- **Memory Efficiency**: Managing memory usage during continuous operation

### Whisper in Noisy Environments

Humanoid robots often operate in noisy environments where traditional ASR systems struggle. Whisper's training on diverse acoustic conditions makes it more robust than conventional systems, but additional techniques can further improve performance:

**Acoustic Preprocessing**: Enhancing audio quality before processing:
- **Noise Reduction**: Filtering background noise using digital signal processing
- **Beamforming**: Using multiple microphones to focus on speaker
- **Echo Cancellation**: Removing acoustic feedback and echoes
- **Automatic Gain Control**: Maintaining consistent audio levels

**Post-processing**: Improving transcription quality after Whisper processing:
- **Language Model Integration**: Using language models to correct errors
- **Context-Based Correction**: Using conversation context for error correction
- **Confidence Scoring**: Assessing transcription confidence for reliability
- **Error Recovery**: Handling low-confidence transcriptions appropriately

## Large Language Models (LLMs) for Robot Planning

### LLM Integration with Robotics

Large Language Models provide the reasoning and planning capabilities necessary to convert natural language commands into executable robotic actions. These models excel at understanding the semantic meaning of commands, breaking complex instructions into manageable subtasks, and generating appropriate responses to user queries. In humanoid robotics, LLMs serve as the "brain" that interprets user commands and plans appropriate robot behaviors.

The integration of LLMs with robotics systems involves several key components:

**Prompt Engineering**: Crafting inputs to elicit appropriate responses from LLMs:
- **System Prompts**: Defining the robot's role and capabilities
- **Context Injection**: Providing environmental and robot state information
- **Example Responses**: Demonstrating expected output formats
- **Constraint Enforcement**: Guiding responses toward executable actions

**Output Parsing**: Converting LLM responses into robot commands:
- **Structured Output**: Requiring LLMs to generate parseable formats
- **Action Extraction**: Identifying specific actions from text
- **Parameter Extraction**: Identifying parameters and arguments
- **Validation**: Verifying that extracted actions are valid

### Task Decomposition and Planning

LLMs excel at decomposing complex natural language commands into sequences of simpler actions:

**Hierarchical Planning**: Breaking tasks into hierarchical subtasks:
- **High-level Goals**: Understanding overall task objectives
- **Mid-level Actions**: Decomposing goals into actionable steps
- **Low-level Commands**: Generating specific robot commands
- **Temporal Coordination**: Sequencing actions appropriately

**Constraint Handling**: Managing environmental and robot constraints:
- **Physical Constraints**: Respecting robot kinematic and dynamic limits
- **Environmental Constraints**: Respecting environmental restrictions
- **Safety Constraints**: Ensuring actions are safe to execute
- **Temporal Constraints**: Managing timing requirements

### LLM Specialization for Robotics

While general-purpose LLMs provide strong language understanding capabilities, they require specialization for effective robotics integration:

**Robotics Domain Knowledge**: Teaching LLMs about robotics concepts:
- **Robot Kinematics**: Understanding robot movement capabilities
- **Task Categories**: Recognizing different types of robotic tasks
- **Environmental Interaction**: Understanding object manipulation and navigation
- **Safety Considerations**: Recognizing potentially dangerous commands

**Action Space Definition**: Constraining LLM outputs to robot capabilities:
- **Available Actions**: Defining the robot's action vocabulary
- **Action Parameters**: Defining valid parameter ranges
- **Action Sequencing**: Defining how actions can be combined
- **Error Handling**: Defining responses to invalid commands

## Vision-Language-Action (VLA) Systems

### VLA Fundamentals

Vision-Language-Action (VLA) systems represent a paradigm where robots can understand visual scenes, interpret language commands, and execute corresponding actions. This integration creates a unified framework for natural human-robot interaction where users can refer to objects and locations in the robot's environment using natural language. For humanoid robots, VLA systems enable complex interaction scenarios where the robot must perceive its environment, understand spoken commands, and execute appropriate actions.

VLA systems typically consist of multiple interconnected neural networks:
- **Vision Network**: Processing visual input to understand the scene
- **Language Network**: Processing textual input to understand commands
- **Action Network**: Generating motor commands for the robot
- **Fusion Mechanism**: Combining visual and language information

### RT-2: Robot Learning with Vision-Language-Action Reasoning

RT-2 (Robotics Transformer 2) represents a significant advancement in VLA systems, using large-scale pretraining to enable robots to understand and execute language commands in the context of visual scenes. The system learns to map high-level language commands to low-level robot actions by training on large datasets of internet text, images, and robot demonstrations.

**Key Capabilities of RT-2**:
- **Generalization**: Understanding novel commands not seen during training
- **Scene Understanding**: Interpreting visual scenes and identifying objects
- **Action Grounding**: Mapping language concepts to robot actions
- **Reasoning**: Performing multi-step reasoning for complex tasks

### VLA Integration in Humanoid Robots

Integrating VLA systems into humanoid robots involves several architectural considerations:

**Multi-modal Processing**: Handling visual and linguistic inputs:
- **Visual Processing Pipeline**: Processing camera feeds for scene understanding
- **Language Processing Pipeline**: Processing natural language commands
- **Cross-modal Attention**: Attending to relevant visual elements based on language
- **Temporal Integration**: Maintaining scene understanding over time

**Action Generation**: Converting multi-modal inputs to robot actions:
- **Skill Selection**: Choosing appropriate robot skills for commands
- **Parameter Generation**: Generating specific parameters for actions
- **Trajectory Planning**: Planning detailed motor trajectories
- **Safety Verification**: Ensuring planned actions are safe

## Implementation Architecture for Voice-Enabled Humanoid Robots

### System Architecture Overview

The complete architecture for voice-enabled humanoid robots integrates multiple components that must work together seamlessly:

**Audio Processing Layer**: Handling speech input and preprocessing:
- **Microphone Array**: Capturing audio from the environment
- **Audio Preprocessing**: Filtering and enhancing audio quality
- **Voice Activity Detection**: Identifying when speech is occurring
- **Audio Buffering**: Managing streaming audio for processing

**Speech Recognition Layer**: Converting speech to text:
- **Whisper Integration**: Running Whisper for speech-to-text conversion
- **Transcription Management**: Managing transcribed text quality
- **Confidence Assessment**: Evaluating transcription confidence
- **Error Handling**: Managing recognition errors

**Language Understanding Layer**: Processing natural language commands:
- **LLM Integration**: Running LLMs for command interpretation
- **Context Management**: Maintaining conversation and task context
- **Intent Recognition**: Identifying user intentions from commands
- **Entity Resolution**: Identifying objects, locations, and parameters

**Action Planning Layer**: Converting commands to robot actions:
- **Task Decomposition**: Breaking complex commands into subtasks
- **Action Mapping**: Mapping language concepts to robot capabilities
- **Motion Planning**: Generating detailed robot trajectories
- **Safety Validation**: Ensuring planned actions are safe

**Execution Layer**: Executing robot actions:
- **Motion Control**: Controlling robot joints and actuators
- **Feedback Integration**: Incorporating sensor feedback during execution
- **Error Recovery**: Handling execution failures
- **User Communication**: Providing status updates

### Real-time Processing Considerations

Voice-enabled humanoid robots must maintain real-time performance for natural interaction:

**Latency Management**: Minimizing delay between speech and action:
- **Pipeline Optimization**: Optimizing each processing stage
- **Parallel Processing**: Running independent tasks in parallel
- **Caching**: Caching intermediate results when possible
- **Prioritization**: Prioritizing critical tasks for immediate processing

**Resource Allocation**: Managing computational resources efficiently:
- **GPU Scheduling**: Scheduling GPU tasks for optimal utilization
- **Memory Management**: Efficiently managing memory for streaming processing
- **Load Balancing**: Distributing computational load appropriately
- **Power Management**: Optimizing for power efficiency when battery-powered

## Practical Implementation Examples

### Whisper Integration Example

A practical implementation of Whisper for humanoid robotics might include the following components:

**Audio Input Management**: Handling microphone input with appropriate preprocessing:
- **Multiple Channels**: Managing multi-channel audio from microphone arrays
- **Noise Filtering**: Applying real-time noise reduction algorithms
- **Automatic Gain Control**: Maintaining consistent audio levels
- **Voice Activity Detection**: Identifying speech segments for processing

**Streaming Processing**: Implementing real-time audio processing:
- **Sliding Windows**: Processing audio in overlapping windows
- **Buffer Management**: Efficiently managing audio buffers
- **Chunk Processing**: Breaking long utterances into processable chunks
- **Context Preservation**: Maintaining context across chunks

### LLM Integration for Task Planning

Integrating LLMs for task planning requires careful prompt engineering and output processing:

**Context-Aware Prompts**: Providing environmental context to LLMs:
- **Robot State**: Current robot position, configuration, and status
- **Environment State**: Object locations, obstacles, and environmental conditions
- **Task History**: Previous interactions and current task progress
- **Capability Information**: Available robot actions and constraints

**Structured Output**: Ensuring LLMs generate parseable responses:
- **JSON Output**: Requiring structured JSON responses
- **Action Schema**: Defining the expected action format
- **Validation Rules**: Defining constraints on LLM outputs
- **Error Recovery**: Defining responses to invalid outputs

## Safety and Reliability Considerations

### Safety Architecture

Voice-enabled humanoid robots must implement robust safety measures:

**Command Validation**: Verifying that voice commands are safe to execute:
- **Safety Filters**: Filtering out dangerous commands before execution
- **Constraint Checking**: Verifying commands against safety constraints
- **Context Validation**: Ensuring commands are appropriate for current context
- **Human Oversight**: Providing mechanisms for human intervention

**Error Handling**: Managing system failures gracefully:
- **Recognition Errors**: Handling speech recognition failures
- **Understanding Errors**: Handling command misinterpretation
- **Execution Errors**: Managing robot action failures
- **Recovery Procedures**: Implementing recovery from errors

### Reliability Engineering

Ensuring reliable operation of voice-enabled systems:

**Performance Monitoring**: Continuously monitoring system performance:
- **Recognition Accuracy**: Tracking speech recognition performance
- **Response Time**: Monitoring system response latency
- **Failure Rates**: Tracking various types of system failures
- **Resource Usage**: Monitoring computational and memory usage

**Degradation Management**: Handling system degradation gracefully:
- **Fallback Mechanisms**: Providing alternative operation modes
- **Quality Scaling**: Adjusting processing quality based on available resources
- **Priority Management**: Prioritizing critical functions during resource constraints
- **User Notification**: Informing users of system limitations

## Advanced Topics in Voice-Enabled Robotics

### Multimodal Interaction

Advanced voice-enabled robots can integrate multiple interaction modalities:

**Gesture Integration**: Combining voice and gesture input:
- **Gesture Recognition**: Recognizing pointing, waving, and other gestures
- **Multimodal Fusion**: Combining gesture and voice information
- **Ambiguity Resolution**: Using gestures to clarify voice commands
- **Natural Interaction**: Creating more natural human-robot interaction

**Visual Feedback**: Providing visual confirmation of voice commands:
- **Attention Indication**: Showing where robot is looking or pointing
- **Action Visualization**: Visualizing planned actions before execution
- **Status Indication**: Showing robot state and understanding
- **Error Visualization**: Visualizing errors and system status

### Context-Aware Systems

Advanced systems maintain and use context for more natural interaction:

**Conversation Context**: Maintaining conversation history and context:
- **Pronoun Resolution**: Understanding references to previously mentioned objects
- **Topic Maintenance**: Maintaining focus on current conversation topics
- **Implicit Commands**: Understanding commands that rely on context
- **Memory Management**: Managing long-term and short-term context

**Environmental Context**: Understanding and adapting to environmental conditions:
- **Object Tracking**: Tracking objects mentioned in conversation
- **Spatial Reasoning**: Understanding spatial relationships and locations
- **Activity Recognition**: Recognizing ongoing activities and events
- **Context Prediction**: Predicting likely user needs based on context

## Integration Challenges and Solutions

### System Integration Challenges

Integrating voice recognition, LLMs, and robot control presents several challenges:

**Timing Coordination**: Coordinating real-time processing with control systems:
- **Control Loop Integration**: Integrating voice processing with control loops
- **Timing Guarantees**: Ensuring real-time performance requirements
- **Buffer Synchronization**: Synchronizing audio, video, and control buffers
- **Latency Management**: Minimizing end-to-end system latency

**Data Flow Management**: Managing data flow between different system components:
- **Message Passing**: Efficiently passing data between components
- **Data Format Conversion**: Converting between different data formats
- **Bandwidth Management**: Managing network and computational bandwidth
- **Resource Sharing**: Sharing computational resources between components

### Performance Optimization

Optimizing performance across the entire system:

**Pipeline Optimization**: Optimizing the complete processing pipeline:
- **Component Optimization**: Optimizing individual components
- **Inter-component Optimization**: Optimizing communication between components
- **End-to-end Optimization**: Optimizing the complete pipeline
- **Adaptive Optimization**: Adjusting optimization based on current conditions

**Hardware Acceleration**: Leveraging hardware acceleration appropriately:
- **GPU Utilization**: Using GPUs for computationally intensive tasks
- **Specialized Hardware**: Using specialized hardware for specific tasks
- **Memory Bandwidth**: Optimizing for memory bandwidth limitations
- **Power Efficiency**: Balancing performance with power consumption

## Future Directions and Research Areas

### Emerging Technologies

Several emerging technologies are advancing voice-enabled robotics:

**Foundation Models**: Large foundation models for robotics:
- **Multimodal Foundation Models**: Models that understand vision, language, and action
- **Robot-Specific Pretraining**: Pretraining on large robotics datasets
- **Transfer Learning**: Transferring knowledge from one robot to another
- **Few-shot Learning**: Learning new tasks from minimal demonstrations

**Edge AI**: Bringing AI capabilities to edge devices:
- **Model Compression**: Compressing models for edge deployment
- **Federated Learning**: Learning across distributed robot systems
- **On-device Inference**: Running models on robot hardware
- **Privacy Preservation**: Maintaining privacy in edge systems

### Research Challenges

Several research challenges remain in voice-enabled robotics:

**Robustness**: Improving robustness to various conditions:
- **Acoustic Robustness**: Operating in various acoustic conditions
- **Visual Robustness**: Operating in various lighting and visual conditions
- **Linguistic Robustness**: Understanding various language patterns
- **Environmental Robustness**: Operating in various environmental conditions

**Learning and Adaptation**: Enabling robots to learn and adapt:
- **Online Learning**: Learning from ongoing interactions
- **Transfer Learning**: Transferring knowledge between tasks
- **Personalization**: Adapting to individual users
- **Curriculum Learning**: Learning complex tasks incrementally

## Conclusion

The integration of Whisper for speech recognition, LLMs for language understanding, and VLA systems for action planning represents a significant advancement in human-robot interaction for humanoid robots. These technologies enable natural, intuitive interaction through spoken language while providing the reasoning and planning capabilities necessary for complex task execution.

The success of voice-enabled humanoid robots depends on careful integration of multiple technologies, attention to real-time performance requirements, and robust safety measures. As these technologies continue to advance, we can expect even more sophisticated and capable voice-enabled robotic systems that can truly understand and assist humans in natural ways.

The next chapter will explore the capstone topic of humanoid autonomy, bringing together all the concepts covered throughout the book to create integrated autonomous humanoid systems.

---

## References

[1] Radford, A., Kim, J. W., Xu, T., Brockman, G., McLeavey, C., & Sutskever, I. (2022). Robust speech recognition via large-scale weak supervision. *arXiv preprint arXiv:2212.04356*.

[2] Brohan, A., Brown, D., Carbajal, J., Chebotar, Y., D'Agostaro, M., Dapello, J., ... & Bousmalis, K. (2022). RT-1: Robotics transformer for real-world control at scale. *arXiv preprint arXiv:2212.06817*.

[3] Driess, D., Xu, Z., Srinivasa, S., & Lee, J. (2023). RT-2: Vision-language-action models transfer web knowledge to robotic control. *arXiv preprint arXiv:2307.15818*.

[4] Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics*. Springer.

[5] OpenAI. (2023). Whisper API Documentation. Retrieved from https://platform.openai.com/docs/guides/speech-to-text

## Code Examples

### Example 1: Whisper Integration for Robot Voice Commands
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Twist
import whisper
import torch
import numpy as np
import queue
import threading
import time

class WhisperRobotCommander(Node):
    def __init__(self):
        super().__init__('whisper_robot_commander')

        # Load Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base")  # Use "base" for real-time performance
        self.get_logger().info('Whisper model loaded')

        # Create audio queue for processing
        self.audio_queue = queue.Queue()
        self.command_queue = queue.Queue()

        # Create subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio',
            self.audio_callback,
            10
        )

        # Create publishers
        self.command_pub = self.create_publisher(
            String,
            '/voice_commands',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Start audio processing thread
        self.processing_thread = threading.Thread(target=self.process_audio_stream)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Robot state
        self.robot_state = {
            'location': 'kitchen',
            'battery': 0.85,
            'current_task': None
        }

        self.get_logger().info('Whisper Robot Commander initialized')

    def audio_callback(self, msg):
        """Handle incoming audio data"""
        # Convert audio data to numpy array
        audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32)
        audio_array = audio_array / 32768.0  # Normalize to [-1, 1]

        # Add to processing queue
        self.audio_queue.put(audio_array)

    def process_audio_stream(self):
        """Process audio stream for speech recognition"""
        audio_buffer = np.array([])
        buffer_size = 16000 * 3  # 3 seconds of audio at 16kHz

        while rclpy.ok():
            try:
                # Get audio chunk
                chunk = self.audio_queue.get(timeout=1.0)

                # Add to buffer
                audio_buffer = np.concatenate([audio_buffer, chunk])

                # If buffer is large enough, process it
                if len(audio_buffer) >= buffer_size:
                    # Process audio with Whisper
                    result = self.transcribe_audio(audio_buffer)

                    if result and result.text.strip():
                        # Clean up the transcription
                        command = result.text.strip().lower()

                        # Add to command queue for processing
                        self.command_queue.put(command)

                        # Log the recognized command
                        self.get_logger().info(f'Recognized: "{command}"')

                    # Keep some overlap for continuous processing
                    audio_buffer = audio_buffer[-int(16000 * 1.0):]  # Keep 1 second overlap

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in audio processing: {e}')

    def transcribe_audio(self, audio_array):
        """Transcribe audio using Whisper"""
        try:
            # Convert to 16kHz if needed
            if len(audio_array) > 0:
                # Process with Whisper
                result = self.model.transcribe(
                    audio_array,
                    language='en',
                    fp16=torch.cuda.is_available()
                )
                return result
        except Exception as e:
            self.get_logger().error(f'Error in Whisper transcription: {e}')
            return None

        return None

    def process_voice_command(self, command):
        """Process recognized voice command"""
        # Parse and execute command
        parsed_command = self.parse_command(command)

        if parsed_command:
            self.execute_command(parsed_command)
            return True

        return False

    def parse_command(self, command):
        """Parse natural language command into robot action"""
        # Simple command parser - in real implementation, use LLM
        command_lower = command.lower()

        # Navigation commands
        if 'go to' in command_lower or 'move to' in command_lower:
            if 'kitchen' in command_lower:
                return {'action': 'navigate', 'target': 'kitchen', 'location': 'kitchen'}
            elif 'living room' in command_lower:
                return {'action': 'navigate', 'target': 'living_room', 'location': 'living_room'}
            elif 'bedroom' in command_lower:
                return {'action': 'navigate', 'target': 'bedroom', 'location': 'bedroom'}

        # Movement commands
        elif 'move forward' in command_lower or 'go forward' in command_lower:
            return {'action': 'move', 'direction': 'forward', 'distance': 1.0}
        elif 'turn left' in command_lower:
            return {'action': 'turn', 'direction': 'left', 'angle': 90.0}
        elif 'turn right' in command_lower:
            return {'action': 'turn', 'direction': 'right', 'angle': 90.0}
        elif 'stop' in command_lower:
            return {'action': 'stop'}

        # Object interaction
        elif 'pick up' in command_lower or 'grasp' in command_lower:
            if 'cup' in command_lower:
                return {'action': 'grasp', 'object': 'cup', 'object_type': 'cup'}
            elif 'book' in command_lower:
                return {'action': 'grasp', 'object': 'book', 'object_type': 'book'}

        # Information requests
        elif 'where are you' in command_lower:
            return {'action': 'report_location'}
        elif 'battery' in command_lower or 'charge' in command_lower:
            return {'action': 'report_battery'}

        # If no recognized command, return None
        return None

    def execute_command(self, parsed_command):
        """Execute parsed command on robot"""
        action = parsed_command.get('action')

        if action == 'navigate':
            self.execute_navigation(parsed_command)
        elif action == 'move':
            self.execute_movement(parsed_command)
        elif action == 'turn':
            self.execute_turn(parsed_command)
        elif action == 'grasp':
            self.execute_grasp(parsed_command)
        elif action == 'stop':
            self.execute_stop()
        elif action == 'report_location':
            self.report_location()
        elif action == 'report_battery':
            self.report_battery()
        else:
            self.get_logger().warn(f'Unknown command action: {action}')

    def execute_navigation(self, command):
        """Execute navigation command"""
        target = command.get('location', 'unknown')

        # In real implementation, send navigation goal to Nav2
        self.get_logger().info(f'Navigating to {target}')

        # Publish command for higher-level navigation system
        nav_cmd = String()
        nav_cmd.data = f'navigate_to_{target}'
        self.command_pub.publish(nav_cmd)

    def execute_movement(self, command):
        """Execute movement command"""
        direction = command.get('direction', 'forward')
        distance = command.get('distance', 1.0)

        # Send velocity command
        cmd_vel = Twist()
        if direction == 'forward':
            cmd_vel.linear.x = 0.3  # m/s
        elif direction == 'backward':
            cmd_vel.linear.x = -0.3

        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Moving {direction} for {distance}m')

    def execute_turn(self, command):
        """Execute turn command"""
        direction = command.get('direction', 'left')
        angle = command.get('angle', 90.0)

        # Send rotation command
        cmd_vel = Twist()
        if direction == 'left':
            cmd_vel.angular.z = 0.5  # rad/s
        elif direction == 'right':
            cmd_vel.angular.z = -0.5

        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Turning {direction} by {angle} degrees')

    def execute_grasp(self, command):
        """Execute grasp command"""
        obj_type = command.get('object_type', 'unknown')
        self.get_logger().info(f'Attempting to grasp {obj_type}')

        # In real implementation, send grasp command to manipulation system
        grasp_cmd = String()
        grasp_cmd.data = f'grasp_{obj_type}'
        self.command_pub.publish(grasp_cmd)

    def execute_stop(self):
        """Execute stop command"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Robot stopped')

    def report_location(self):
        """Report current location"""
        location = self.robot_state.get('location', 'unknown')
        report = String()
        report.data = f'Robot is currently in the {location}'
        self.command_pub.publish(report)
        self.get_logger().info(f'Location report: {report.data}')

    def report_battery(self):
        """Report battery status"""
        battery = self.robot_state.get('battery', 0.0)
        battery_percent = int(battery * 100)
        report = String()
        report.data = f'Battery level is {battery_percent}%'
        self.command_pub.publish(report)
        self.get_logger().info(f'Battery report: {report.data}')

    def run_command_processing(self):
        """Main command processing loop"""
        while rclpy.ok():
            try:
                # Get command from queue
                command = self.command_queue.get(timeout=0.1)

                # Process the command
                success = self.process_voice_command(command)

                if not success:
                    self.get_logger().warn(f'Failed to process command: {command}')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in command processing: {e}')

def main(args=None):
    rclpy.init(args=args)
    commander = WhisperRobotCommander()

    # Run command processing in main thread
    commander.run_command_processing()

    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: LLM Integration for Task Planning
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import openai
import json
import time
import asyncio
from concurrent.futures import ThreadPoolExecutor

class LLMTaskPlanner(Node):
    def __init__(self):
        super().__init__('llm_task_planner')

        # OpenAI API configuration
        # Note: In production, use environment variables for API keys
        openai.api_key = "your-openai-api-key-here"

        # Create subscribers
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.command_callback,
            10
        )

        # Create publishers
        self.task_pub = self.create_publisher(
            String,
            '/planned_tasks',
            10
        )

        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Robot capabilities
        self.robot_capabilities = {
            "navigation": ["navigate_to_location", "move_forward", "turn_left", "turn_right", "stop"],
            "manipulation": ["grasp_object", "place_object", "open_gripper", "close_gripper"],
            "interaction": ["speak", "display_message", "listen"],
            "perception": ["detect_objects", "localize_robot", "map_environment"]
        }

        # Environment information
        self.environment_map = {
            "locations": {
                "kitchen": {"x": 1.0, "y": 1.0, "theta": 0.0},
                "living_room": {"x": 5.0, "y": 2.0, "theta": 0.0},
                "bedroom": {"x": 3.0, "y": 4.0, "theta": 1.57},
                "office": {"x": 6.0, "y": 1.0, "theta": 3.14}
            },
            "objects": {
                "cup": {"location": "kitchen", "type": "graspable"},
                "book": {"location": "living_room", "type": "graspable"},
                "chair": {"location": "living_room", "type": "movable"}
            }
        }

        # Task execution queue
        self.task_queue = []
        self.current_task = None

        self.get_logger().info('LLM Task Planner initialized')

    def command_callback(self, msg):
        """Handle incoming voice commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Plan task using LLM
        asyncio.create_task(self.plan_task_async(command))

    async def plan_task_async(self, command):
        """Plan task asynchronously using LLM"""
        try:
            # Create task planning prompt
            prompt = self.create_planning_prompt(command)

            # Call OpenAI API
            response = await self.call_llm_async(prompt)

            if response:
                # Parse the response
                planned_tasks = self.parse_llm_response(response)

                if planned_tasks:
                    # Execute the planned tasks
                    await self.execute_planned_tasks(planned_tasks)

        except Exception as e:
            self.get_logger().error(f'Error in task planning: {e}')

    def create_planning_prompt(self, command):
        """Create prompt for LLM task planning"""
        prompt = f"""
You are a task planning assistant for a humanoid robot. Your job is to convert natural language commands into sequences of robot actions.

Robot capabilities:
- Navigation: {self.robot_capabilities['navigation']}
- Manipulation: {self.robot_capabilities['manipulation']}
- Interaction: {self.robot_capabilities['interaction']}
- Perception: {self.robot_capabilities['perception']}

Environment map:
- Locations: {list(self.environment_map['locations'].keys())}
- Objects: {list(self.environment_map['objects'].keys())}

Current command: "{command}"

Please respond with a JSON array of tasks to execute, following this exact format:
[
    {{
        "action": "action_name",
        "parameters": {{
            "param1": "value1",
            "param2": "value2"
        }},
        "description": "Brief description of what this step does"
    }}
]

Each action must be from the robot capabilities list. Only return the JSON array, nothing else.
"""
        return prompt

    async def call_llm_async(self, prompt):
        """Call LLM API asynchronously"""
        loop = asyncio.get_event_loop()

        with ThreadPoolExecutor() as executor:
            try:
                response = await loop.run_in_executor(
                    executor,
                    openai.ChatCompletion.create,
                    {
                        "model": "gpt-3.5-turbo",
                        "messages": [{"role": "user", "content": prompt}],
                        "temperature": 0.1,  # Low temperature for consistent output
                        "max_tokens": 500
                    }
                )

                return response.choices[0].message.content

            except Exception as e:
                self.get_logger().error(f'LLM API call failed: {e}')
                return None

    def parse_llm_response(self, response_text):
        """Parse LLM response into structured task list"""
        try:
            # Clean up the response (remove any text before/after JSON)
            json_start = response_text.find('[')
            json_end = response_text.rfind(']') + 1

            if json_start != -1 and json_end != 0:
                json_str = response_text[json_start:json_end]
                tasks = json.loads(json_str)

                # Validate tasks
                for task in tasks:
                    if 'action' not in task or 'parameters' not in task:
                        raise ValueError("Invalid task format")

                self.get_logger().info(f'Parsed {len(tasks)} tasks from LLM response')
                return tasks

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response as JSON: {e}')
            self.get_logger().debug(f'LLM response: {response_text}')
        except Exception as e:
            self.get_logger().error(f'Error parsing LLM response: {e}')

        return []

    async def execute_planned_tasks(self, tasks):
        """Execute planned tasks sequentially"""
        for task in tasks:
            await self.execute_single_task(task)

    async def execute_single_task(self, task):
        """Execute a single task"""
        action = task.get('action', '')
        parameters = task.get('parameters', {})
        description = task.get('description', 'No description')

        self.get_logger().info(f'Executing task: {action} - {description}')

        # Execute based on action type
        if action == 'navigate_to_location':
            await self.execute_navigation(parameters)
        elif action == 'grasp_object':
            await self.execute_grasp(parameters)
        elif action == 'move_forward':
            await self.execute_move_forward(parameters)
        elif action == 'turn_left':
            await self.execute_turn_left(parameters)
        elif action == 'turn_right':
            await self.execute_turn_right(parameters)
        elif action == 'speak':
            await self.execute_speak(parameters)
        else:
            self.get_logger().warn(f'Unknown action: {action}')

        # Publish task completion
        task_result = String()
        task_result.data = f'task_completed:{action}'
        self.task_pub.publish(task_result)

        self.get_logger().info(f'Completed task: {action}')

    async def execute_navigation(self, parameters):
        """Execute navigation task"""
        location_name = parameters.get('location', 'unknown')

        if location_name in self.environment_map['locations']:
            location_data = self.environment_map['locations'][location_name]

            # Create navigation goal
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = location_data['x']
            goal_pose.pose.position.y = location_data['y']
            goal_pose.pose.position.z = 0.0

            # Convert theta to quaternion (simplified)
            theta = location_data['theta']
            goal_pose.pose.orientation.z = theta
            goal_pose.pose.orientation.w = 1.0

            # Publish navigation goal
            self.nav_goal_pub.publish(goal_pose)
            self.get_logger().info(f'Published navigation goal to {location_name}')
        else:
            self.get_logger().warn(f'Unknown location: {location_name}')

    async def execute_grasp(self, parameters):
        """Execute grasp task"""
        object_name = parameters.get('object', 'unknown')

        if object_name in self.environment_map['objects']:
            object_data = self.environment_map['objects'][object_name]
            self.get_logger().info(f'Attempting to grasp {object_name} from {object_data["location"]}')

            # In real implementation, send grasp command to manipulation system
            grasp_cmd = String()
            grasp_cmd.data = f'grasp_{object_name}'
            self.task_pub.publish(grasp_cmd)
        else:
            self.get_logger().warn(f'Unknown object: {object_name}')

    async def execute_move_forward(self, parameters):
        """Execute move forward task"""
        distance = parameters.get('distance', 1.0)
        self.get_logger().info(f'Moving forward {distance} meters')

        # In real implementation, send movement command
        move_cmd = String()
        move_cmd.data = f'move_forward_{distance}'
        self.task_pub.publish(move_cmd)

    async def execute_turn_left(self, parameters):
        """Execute turn left task"""
        angle = parameters.get('angle', 90.0)
        self.get_logger().info(f'Turning left by {angle} degrees')

        # In real implementation, send turn command
        turn_cmd = String()
        turn_cmd.data = f'turn_left_{angle}'
        self.task_pub.publish(turn_cmd)

    async def execute_turn_right(self, parameters):
        """Execute turn right task"""
        angle = parameters.get('angle', 90.0)
        self.get_logger().info(f'Turning right by {angle} degrees')

        # In real implementation, send turn command
        turn_cmd = String()
        turn_cmd.data = f'turn_right_{angle}'
        self.task_pub.publish(turn_cmd)

    async def execute_speak(self, parameters):
        """Execute speak task"""
        text = parameters.get('text', 'Hello')
        self.get_logger().info(f'Speaking: {text}')

        # In real implementation, send to text-to-speech system
        speak_cmd = String()
        speak_cmd.data = f'speak:{text}'
        self.task_pub.publish(speak_cmd)

def main(args=None):
    rclpy.init(args=args)
    planner = LLMTaskPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info('Shutting down LLM Task Planner')
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: VLA System Integration
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np
import cv2
import torch
import torchvision.transforms as transforms
from transformers import CLIPProcessor, CLIPModel
import groundingdino.datasets.transforms as T
from groundingdino.util.inference import predict, load_model

class VLARobotSystem(Node):
    def __init__(self):
        super().__init__('vla_robot_system')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Load CLIP model for vision-language understanding
        self.get_logger().info('Loading CLIP model...')
        self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.get_logger().info('CLIP model loaded')

        # Load GroundingDINO for object detection
        self.get_logger().info('Loading GroundingDINO model...')
        try:
            self.grounding_dino_model = load_model(
                "groundingdino/config/GroundingDINO_SwinT_OGC.py",
                "weights/groundingdino_swint_ogc.pth"
            )
            self.get_logger().info('GroundingDINO model loaded')
        except:
            self.get_logger().warn('GroundingDINO model not available, using basic detection')
            self.grounding_dino_model = None

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.command_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        self.vla_action_pub = self.create_publisher(
            String,
            '/vla_actions',
            10
        )

        # Store latest image and camera info
        self.latest_image = None
        self.camera_info = None
        self.command_queue = []

        # Robot state
        self.robot_state = {
            'current_task': None,
            'detected_objects': [],
            'environment_map': {}
        }

        self.get_logger().info('VLA Robot System initialized')

    def camera_info_callback(self, msg):
        """Update camera information"""
        self.camera_info = msg

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def command_callback(self, msg):
        """Handle voice commands for VLA system"""
        command = msg.data
        self.get_logger().info(f'Received VLA command: {command}')

        # Process command using VLA system
        self.process_vla_command(command)

    def process_vla_command(self, command):
        """Process command using vision-language-action system"""
        if self.latest_image is None:
            self.get_logger().warn('No image available for VLA processing')
            return

        # Detect objects in the scene
        detections = self.detect_objects_in_scene()

        # Match command with detected objects
        action = self.match_command_to_objects(command, detections)

        if action:
            # Execute the action
            self.execute_vla_action(action, detections)
        else:
            self.get_logger().warn(f'Could not match command to objects: {command}')

    def detect_objects_in_scene(self):
        """Detect objects in the current scene using vision system"""
        if self.latest_image is None:
            return []

        # Convert image to RGB format for processing
        image_rgb = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB)

        if self.grounding_dino_model:
            # Use GroundingDINO for object detection
            boxes, logits, phrases = predict(
                model=self.grounding_dino_model,
                image=image_rgb,
                caption="person . cup . book . chair . table . bottle . tv . laptop .",
                box_threshold=0.3,
                text_threshold=0.25
            )

            detections = []
            for box, logit, phrase in zip(boxes, logits, phrases):
                x1, y1, x2, y2 = box
                detection = {
                    'label': phrase,
                    'confidence': float(logit),
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'center': [(x1 + x2) / 2, (y1 + y2) / 2]
                }
                detections.append(detection)
        else:
            # Fallback: use basic detection (this is simplified)
            detections = self.fallback_object_detection(image_rgb)

        # Update robot state with detected objects
        self.robot_state['detected_objects'] = detections

        # Publish detections
        self.publish_detections(detections)

        return detections

    def fallback_object_detection(self, image):
        """Fallback object detection if GroundingDINO is not available"""
        # This is a simplified detection system for demonstration
        detections = []

        # Convert to grayscale for simple blob detection
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Apply threshold to find objects
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        # Find contours (potential objects)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Filter by size to get meaningful objects
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)

                # Simple labeling based on position and size
                if w > h * 2:
                    label = "object"
                else:
                    label = "object"

                detection = {
                    'label': label,
                    'confidence': 0.5,  # Default confidence
                    'bbox': [float(x), float(y), float(x+w), float(y+h)],
                    'center': [float(x + w/2), float(y + h/2)]
                }
                detections.append(detection)

        return detections

    def publish_detections(self, detections):
        """Publish object detections to ROS topic"""
        detection_msg = Detection2DArray()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = 'camera_rgb_optical_frame'

        for detection in detections:
            detection_2d = Detection2D()
            detection_2d.header = detection_msg.header

            # Set bbox
            bbox = detection['bbox']
            detection_2d.bbox.center.x = (bbox[0] + bbox[2]) / 2
            detection_2d.bbox.center.y = (bbox[1] + bbox[3]) / 2
            detection_2d.bbox.size_x = abs(bbox[2] - bbox[0])
            detection_2d.bbox.size_y = abs(bbox[3] - bbox[1])

            # Add result
            result = ObjectHypothesisWithPose()
            result.id = detection['label']
            result.score = detection['confidence']
            detection_2d.results.append(result)

            detection_msg.detections.append(detection_2d)

        self.detection_pub.publish(detection_msg)

    def match_command_to_objects(self, command, detections):
        """Match voice command to detected objects using vision-language fusion"""
        # This is a simplified matching system
        # In a real implementation, this would use more sophisticated NLP

        command_lower = command.lower()

        # Extract object references from command
        object_keywords = ['cup', 'book', 'chair', 'bottle', 'laptop', 'person']
        target_object = None

        for keyword in object_keywords:
            if keyword in command_lower:
                target_object = keyword
                break

        if not target_object:
            # If no specific object mentioned, look for action words
            if 'pick up' in command_lower or 'grasp' in command_lower:
                # Look for graspable objects
                graspable_types = ['cup', 'book', 'bottle', 'laptop']
                for detection in detections:
                    if detection['label'] in graspable_types:
                        target_object = detection['label']
                        break
            elif 'go to' in command_lower or 'navigate to' in command_lower:
                # Look for locations or large objects
                for detection in detections:
                    if detection['label'] in ['person', 'chair', 'table']:
                        target_object = detection['label']
                        break

        if target_object:
            # Find the specific object in detections
            for detection in detections:
                if detection['label'] == target_object and detection['confidence'] > 0.3:
                    # Determine action based on command
                    if 'pick up' in command_lower or 'grasp' in command_lower:
                        return {
                            'action': 'grasp_object',
                            'object': target_object,
                            'bbox': detection['bbox'],
                            'center': detection['center']
                        }
                    elif 'go to' in command_lower or 'navigate to' in command_lower:
                        return {
                            'action': 'navigate_to_object',
                            'object': target_object,
                            'bbox': detection['bbox'],
                            'center': detection['center']
                        }
                    elif 'look at' in command_lower or 'see' in command_lower:
                        return {
                            'action': 'look_at_object',
                            'object': target_object,
                            'bbox': detection['bbox'],
                            'center': detection['center']
                        }

        return None

    def execute_vla_action(self, action, detections):
        """Execute vision-language-action based on matched command"""
        action_type = action['action']
        target_object = action['object']

        self.get_logger().info(f'Executing VLA action: {action_type} for {target_object}')

        if action_type == 'grasp_object':
            self.execute_grasp_action(action, detections)
        elif action_type == 'navigate_to_object':
            self.execute_navigation_action(action, detections)
        elif action_type == 'look_at_object':
            self.execute_look_action(action, detections)

    def execute_grasp_action(self, action, detections):
        """Execute grasp action"""
        object_name = action['object']
        bbox = action['bbox']
        center = action['center']

        self.get_logger().info(f'Attempting to grasp {object_name} at position {center}')

        # Calculate 3D position if camera info is available
        if self.camera_info:
            # Convert 2D image coordinates to 3D using camera parameters
            # This is a simplified calculation
            x_3d = center[0] - self.camera_info.k[2]  # cx
            y_3d = center[1] - self.camera_info.k[5]  # cy
            # z would come from depth information in a real implementation

            self.get_logger().info(f'Calculated 3D grasp position: ({x_3d}, {y_3d})')
        # Publish grasp command
        grasp_cmd = String()
        grasp_cmd.data = f'grasp_object:{object_name}:{center[0]}:{center[1]}'
        self.vla_action_pub.publish(grasp_cmd)

    def execute_navigation_action(self, action, detections):
        """Execute navigation action"""
        object_name = action['object']
        center = action['center']

        self.get_logger().info(f'Navigating to {object_name} at position {center}')

        # In a real implementation, this would involve:
        # 1. Generating a navigation goal based on object position
        # 2. Planning a path to approach the object
        # 3. Executing the navigation plan

        # For now, publish a simple navigation command
        nav_cmd = String()
        nav_cmd.data = f'navigate_to_object:{object_name}:{center[0]}:{center[1]}'
        self.vla_action_pub.publish(nav_cmd)

    def execute_look_action(self, action, detections):
        """Execute look action"""
        object_name = action['object']
        center = action['center']

        self.get_logger().info(f'Looking at {object_name} at position {center}')

        # Publish look command
        look_cmd = String()
        look_cmd.data = f'look_at_object:{object_name}:{center[0]}:{center[1]}'
        self.vla_action_pub.publish(look_cmd)

    def get_relevant_objects(self, command, detections):
        """Get objects relevant to the command using CLIP"""
        if not detections:
            return []

        # Prepare images of detected objects for CLIP
        relevant_objects = []

        for detection in detections:
            # Extract bounding box
            bbox = detection['bbox']
            x1, y1, x2, y2 = [int(coord) for coord in bbox]

            # Crop image to object
            if self.latest_image is not None:
                h, w = self.latest_image.shape[:2]
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)

                if x2 > x1 and y2 > y1:
                    object_image = self.latest_image[y1:y2, x1:x2]

                    # Use CLIP to score relevance to command
                    try:
                        inputs = self.clip_processor(
                            text=[command],
                            images=[object_image],
                            return_tensors="pt",
                            padding=True
                        )

                        outputs = self.clip_model(**inputs)
                        logits_per_text = outputs.logits_per_text
                        probs = logits_per_text.softmax(dim=-1)

                        relevance_score = probs[0][0].item()

                        if relevance_score > 0.1:  # Threshold for relevance
                            detection['relevance'] = relevance_score
                            relevant_objects.append(detection)
                    except Exception as e:
                        self.get_logger().warn(f'CLIP scoring failed: {e}')

        return relevant_objects

def main(args=None):
    rclpy.init(args=args)
    vla_system = VLARobotSystem()

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        vla_system.get_logger().info('Shutting down VLA Robot System')
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 4: Integrated Voice System Launch File
```xml
<launch>
  <!-- Audio input configuration -->
  <arg name="audio_device" default="plughw:1,0"/>
  <arg name="sample_rate" default="16000"/>
  <arg name="chunk_size" default="1024"/>

  <!-- Whisper configuration -->
  <arg name="whisper_model" default="base"/>
  <arg name="whisper_language" default="en"/>
  <arg name="whisper_temperature" default="0.0"/>

  <!-- Robot configuration -->
  <arg name="robot_namespace" default="/humanoid_robot"/>
  <arg name="use_sim_time" default="false"/>

  <!-- Audio input node -->
  <node pkg="audio_capture" exec="audio_capture_node" name="audio_capture" output="screen">
    <param name="device" value="$(var audio_device)"/>
    <param name="sample_rate" value="$(var sample_rate)"/>
    <param name="chunk" value="$(var chunk_size)"/>
    <param name="encoding" value="audio/x-raw"/>
    <param name="layout" value="interleaved"/>
  </node>

  <!-- Audio preprocessing -->
  <node pkg="audio_common" exec="audio_preprocessing" name="audio_preprocessing" output="screen">
    <param name="noise_reduction" value="true"/>
    <param name="vad_enabled" value="true"/>
    <param name="vad_threshold" value="0.3"/>
    <param name="gain_control" value="true"/>
  </node>

  <!-- Whisper speech recognition -->
  <node pkg="whisper_ros" exec="whisper_node" name="whisper_recognizer" output="screen">
    <param name="model" value="$(var whisper_model)"/>
    <param name="language" value="$(var whisper_language)"/>
    <param name="temperature" value="$(var whisper_temperature)"/>
    <param name="initial_prompt" value="Robot command system"/>
    <param name="suppress_tokens" value="-1"/>
  </node>

  <!-- Voice command parser -->
  <node pkg="humanoid_voice_control" exec="voice_command_parser" name="voice_command_parser" output="screen">
    <param name="robot_capabilities" value="['navigate', 'grasp', 'speak', 'move', 'turn']"/>
    <param name="navigation_locations" value="['kitchen', 'living_room', 'bedroom', 'office']"/>
    <param name="graspable_objects" value="['cup', 'book', 'bottle', 'laptop']"/>
    <param name="command_timeout" value="5.0"/>
  </node>

  <!-- LLM task planner -->
  <node pkg="llm_planning" exec="llm_task_planner" name="llm_task_planner" output="screen">
    <param name="openai_api_key" value="your-api-key-here"/>
    <param name="model" value="gpt-3.5-turbo"/>
    <param name="temperature" value="0.1"/>
    <param name="max_tokens" value="500"/>
    <param name="robot_capabilities" value="$(find-pkg-share llm_planning)/config/robot_capabilities.json"/>
  </node>

  <!-- VLA system -->
  <node pkg="vla_system" exec="vla_robot_system" name="vla_robot_system" output="screen">
    <param name="clip_model" value="openai/clip-vit-base-patch32"/>
    <param name="detection_threshold" value="0.3"/>
    <param name="command_matching_threshold" value="0.5"/>
    <remap from="camera/rgb/image_raw" to="/camera/color/image_raw"/>
    <remap from="camera/rgb/camera_info" to="/camera/color/camera_info"/>
  </node>

  <!-- Text-to-speech -->
  <node pkg="tts_ros" exec="tts_node" name="text_to_speech" output="screen">
    <param name="voice" value="en-US-Wavenet-D"/>
    <param name="speed" value="1.0"/>
    <param name="pitch" value="1.0"/>
  </node>

  <!-- Robot controller -->
  <node pkg="humanoid_control" exec="humanoid_controller" name="humanoid_controller" output="screen">
    <param name="control_frequency" value="100"/>
    <param name="balance_control_enabled" value="true"/>
  </node>

  <!-- Navigation stack -->
  <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>

  <!-- Manipulation stack -->
  <node pkg="manipulation_stack" exec="manipulation_server" name="manipulation_server" output="screen">
    <param name="grasp_approach_distance" value="0.1"/>
    <param name="grasp_depth" value="0.05"/>
    <param name="grasp_height" value="0.02"/>
  </node>

  <!-- Voice feedback -->
  <node pkg="humanoid_voice_control" exec="voice_feedback" name="voice_feedback" output="screen">
    <param name="feedback_types" value="['command_acknowledged', 'task_started', 'task_completed', 'error']"/>
    <param name="confirmation_phrases" value="['OK, I will ', 'Sure, I am ', 'I understand, I will ']"/>
  </node>
</launch>
```

### Example 5: Voice Command Processing with Safety Features
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import threading
from enum import Enum

class SafetyLevel(Enum):
    SAFE = 0
    WARNING = 1
    DANGER = 2
    CRITICAL = 3

class VoiceCommandSafetyFilter(Node):
    def __init__(self):
        super().__init__('voice_command_safety_filter')

        # Create subscribers
        self.command_sub = self.create_subscription(
            String,
            '/unsafe_voice_commands',
            self.command_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # Create publishers
        self.safe_command_pub = self.create_publisher(
            String,
            '/safe_voice_commands',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.safety_status_pub = self.create_publisher(
            String,
            '/safety_status',
            10
        )

        # Safety parameters
        self.safety_distance = 0.5  # meters
        self.emergency_stop_active = False
        self.robot_moving = False
        self.last_command_time = time.time()
        self.command_timeout = 30.0  # seconds

        # Dangerous commands to filter
        self.dangerous_keywords = [
            'jump', 'run', 'fast', 'hurry', 'quickly', 'danger', 'harm', 'injure',
            'break', 'crash', 'fall', 'knock', 'hit', 'destroy', 'damage'
        ]

        # Thread for safety monitoring
        self.safety_thread = threading.Thread(target=self.safety_monitor)
        self.safety_thread.daemon = True
        self.safety_thread.start()

        self.get_logger().info('Voice Command Safety Filter initialized')

    def command_callback(self, msg):
        """Handle incoming voice commands with safety filtering"""
        command = msg.data.lower()
        self.get_logger().info(f'Received command: {command}')

        # Update command time
        self.last_command_time = time.time()

        # Check if command is safe
        if self.is_command_safe(command):
            # Check environment safety
            if self.is_environment_safe():
                # Publish safe command
                safe_cmd = String()
                safe_cmd.data = command
                self.safe_command_pub.publish(safe_cmd)
                self.get_logger().info(f'Command approved: {command}')

                # Update safety status
                status_msg = String()
                status_msg.data = f'command_approved:{command}'
                self.safety_status_pub.publish(status_msg)
            else:
                self.get_logger().warn('Command rejected due to unsafe environment')
                self.publish_safety_alert('environment_unsafe')
        else:
            self.get_logger().warn(f'Command rejected as unsafe: {command}')
            self.publish_safety_alert('unsafe_command')

    def laser_callback(self, msg):
        """Monitor laser scan for obstacles"""
        # Check for obstacles in front of robot
        front_scan_start = len(msg.ranges) // 2 - 30  # ~30 degrees left of center
        front_scan_end = len(msg.ranges) // 2 + 30    # ~30 degrees right of center

        min_distance = float('inf')
        for i in range(front_scan_start, front_scan_end):
            if i < len(msg.ranges) and not (msg.ranges[i] <= 0.0 or msg.ranges[i] >= float('inf')):
                if msg.ranges[i] < min_distance:
                    min_distance = msg.ranges[i]

        if min_distance < self.safety_distance:
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m - emergency stop')
            self.emergency_stop_active = True
            self.publish_emergency_stop()
        else:
            self.emergency_stop_active = False

    def emergency_stop_callback(self, msg):
        """Handle external emergency stop"""
        self.emergency_stop_active = msg.data

    def is_command_safe(self, command):
        """Check if command is safe to execute"""
        # Check for dangerous keywords
        for keyword in self.dangerous_keywords:
            if keyword in command:
                return False

        # Check for potentially dangerous commands
        dangerous_patterns = [
            'go fast',
            'move quickly',
            'run to',
            'jump over',
            'crash into',
            'break ',
            'destroy '
        ]

        for pattern in dangerous_patterns:
            if pattern in command:
                return False

        # Check for navigation to unknown locations
        if 'go to' in command and 'unknown' in command:
            return False

        return True

    def is_environment_safe(self):
        """Check if environment is safe for robot operation"""
        if self.emergency_stop_active:
            return False

        # Additional safety checks can be added here
        # For example: checking for humans in robot path, etc.

        return True

    def safety_monitor(self):
        """Monitor system safety in a separate thread"""
        while rclpy.ok():
            current_time = time.time()

            # Check for command timeout
            if current_time - self.last_command_time > self.command_timeout:
                self.get_logger().warn('Command timeout - stopping robot')
                self.stop_robot()

            # Additional safety checks can be performed here
            time.sleep(0.1)  # Check every 100ms

    def publish_emergency_stop(self):
        """Publish emergency stop command"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish emergency status
        status_msg = String()
        status_msg.data = 'EMERGENCY_STOP_ACTIVATED'
        self.safety_status_pub.publish(status_msg)

    def stop_robot(self):
        """Stop robot movement"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_safety_alert(self, alert_type):
        """Publish safety alert"""
        alert_msg = String()
        alert_msg.data = f'SAFETY_ALERT:{alert_type}'
        self.safety_status_pub.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_filter = VoiceCommandSafetyFilter()

    try:
        rclpy.spin(safety_filter)
    except KeyboardInterrupt:
        safety_filter.get_logger().info('Shutting down safety filter')
    finally:
        safety_filter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```