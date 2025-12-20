---
title: "Chapter 4: Unity - High-Fidelity Interaction & Sensors"
sidebar_position: 2
---

# Chapter 4: Unity - High-Fidelity Interaction & Sensors

## Introduction to Unity for Robotics Simulation

Unity has emerged as a powerful platform for high-fidelity robotics simulation, offering photorealistic rendering, sophisticated physics modeling, and advanced sensor simulation capabilities. Unlike traditional robotics simulators focused primarily on physics accuracy, Unity excels at creating visually compelling environments that closely approximate real-world conditions. This makes it particularly valuable for humanoid robotics applications where visual perception, human-robot interaction, and realistic sensor data are critical components of system development.

The Unity robotics ecosystem, enhanced by tools like Unity ML-Agents and NVIDIA Isaac Sim, provides a comprehensive platform for developing, training, and testing robotic systems. Unity's strength lies in its ability to generate synthetic data with photorealistic quality, which can be used to train computer vision systems, test perception algorithms, and create immersive environments for human-robot interaction studies.

Unity's architecture is built around a component-based entity system where objects are composed of various components that define their behavior, appearance, and interactions. This design philosophy aligns well with robotics concepts where robots are composed of sensors, actuators, and controllers that work together to achieve complex behaviors.

### Unity's Role in the Robotics Pipeline

In the broader robotics development pipeline, Unity serves several critical functions:

1. **Photorealistic Sensor Simulation**: Generating high-quality camera, lidar, and other sensor data that closely matches real-world sensors
2. **Synthetic Data Generation**: Creating large datasets for training machine learning models without the need for physical data collection
3. **Human-Robot Interaction Studies**: Creating immersive environments for studying human-robot interaction scenarios
4. **Algorithm Development and Testing**: Providing a safe, controllable environment for testing complex robotic behaviors
5. **Visualization and Prototyping**: Rapid prototyping of robotic applications with immediate visual feedback

### Unity vs. Traditional Robotics Simulators

While traditional robotics simulators like Gazebo excel at physics accuracy and real-time performance, Unity offers different strengths:

- **Visual Quality**: Unity provides photorealistic rendering with advanced lighting, shadows, and material properties
- **Asset Creation**: Extensive library of 3D models, materials, and environments
- **Animation Systems**: Sophisticated animation and character control systems
- **Scripting Environment**: C# scripting with extensive APIs for custom behavior
- **Cross-Platform Deployment**: Ability to deploy simulations to various platforms including VR/AR

However, Unity also has trade-offs:
- **Computational Requirements**: Higher computational demands for photorealistic rendering
- **Real-time Performance**: May require optimization for real-time robotic control
- **Physics Accuracy**: While capable, may not match specialized physics engines for all scenarios

## Unity Scene Building and Environment Design

### Scene Architecture and GameObjects

Unity scenes are constructed using GameObjects, which serve as containers for various components that define the object's properties and behavior. In robotics applications, GameObjects represent:

- **Robot Models**: The humanoid robot with all its components and joints
- **Environment Objects**: Furniture, walls, floors, and interactive elements
- **Sensors**: Virtual sensors that generate synthetic data
- **Controllers**: Scripted behaviors that control robot actions
- **Cameras**: Virtual cameras that simulate robot vision systems

Each GameObject can contain multiple components that provide specific functionality. For a humanoid robot, a single joint might contain:

- **Rigidbody**: Physics properties for the link
- **Collider**: Collision detection geometry
- **Joint Component**: Constraints defining the joint type and limits
- **Script Component**: Custom control logic
- **Mesh Renderer**: Visual representation
- **Sensors**: Virtual sensors attached to the link

### Environment Design Principles

Creating effective environments for humanoid robotics requires careful attention to several design principles:

**Realism vs. Performance**: Balancing visual fidelity with computational performance to maintain real-time simulation. This involves using appropriate polygon counts, texture resolutions, and lighting complexity.

**Physical Accuracy**: Ensuring that the environment's physical properties (friction, mass, collision geometry) accurately represent real-world conditions for realistic robot interaction.

**Sensor Compatibility**: Designing environments that provide appropriate challenges for robot sensors, including varied lighting conditions, textures, and geometric complexity.

**Interaction Design**: Creating environments with objects and surfaces that allow for meaningful robot interaction and manipulation tasks.

### Advanced Scene Features

Unity offers several advanced features that enhance robotics simulation:

**ProBuilder**: Built-in tools for creating custom 3D geometry without external modeling software
**Terrain System**: Tools for creating large outdoor environments with realistic terrain
**Lighting and Reflection Probes**: Advanced lighting systems that affect sensor simulation
**Occlusion Culling**: Optimization techniques for complex scenes
**LOD (Level of Detail)**: Automatic switching between detailed and simplified models based on distance

## High-Fidelity Rendering and Visual Simulation

### Physically-Based Rendering (PBR)

Unity's Physically-Based Rendering system ensures that materials and lighting behave according to real-world physics principles. This is crucial for robotics applications where the visual appearance of objects directly affects sensor simulation and perception algorithms.

PBR in Unity includes:

- **Metallic-Roughness Workflow**: Materials defined by metallic and roughness properties
- **Specular-Glossiness Workflow**: Alternative material definition for specialized cases
- **Normal Maps**: Detailed surface geometry without high polygon counts
- **Occlusion Maps**: Simulation of light occlusion in complex surfaces
- **Emission Maps**: Self-illuminating surfaces for special effects

For robotics, PBR ensures that synthetic camera data closely matches real-world camera data, making it suitable for training perception systems.

### Advanced Lighting Systems

Unity provides several lighting systems that affect both visual appearance and sensor simulation:

**Real-time Global Illumination**: Dynamic lighting that responds to environmental changes
**Light Probes**: Capturing and interpolating lighting information for moving objects
**Reflection Probes**: Accurate reflection simulation for shiny surfaces
**Shadow Mapping**: Realistic shadow generation with various quality settings

Lighting systems are particularly important for robotics because they affect the quality of synthetic camera data and can be used to simulate various real-world lighting conditions for robust perception system training.

### Post-Processing Effects

Unity's post-processing stack allows for advanced visual effects that can simulate camera properties and environmental conditions:

- **Color Grading**: Adjusting color properties to match real camera characteristics
- **Depth of Field**: Simulating camera focus effects
- **Motion Blur**: Simulating camera motion effects
- **Bloom**: Simulating lens flare and bright light effects
- **Distortion**: Simulating lens distortion effects

These effects can be configured to match the specific characteristics of real sensors, making synthetic data more realistic for training and testing.

## Depth and LiDAR Sensor Simulation

### Camera and Depth Sensor Integration

Unity's rendering pipeline provides the foundation for high-fidelity camera and depth sensor simulation. The engine renders scenes from the robot's perspective, generating realistic images that include proper lighting, shadows, and material properties.

**Camera Components**: Unity cameras can be configured with properties that match real cameras:
- **Field of View**: Matching the real camera's focal length
- **Resolution**: Matching the real camera's pixel dimensions
- **Sensor Size**: Affecting depth of field and perspective
- **Clipping Planes**: Defining the range of visible objects
- **Noise Simulation**: Adding realistic sensor noise patterns

**Depth Sensing**: Unity can generate depth information through:
- **Depth Buffer**: Direct depth information from the rendering pipeline
- **Raycasting**: Precise distance measurements to surfaces
- **Custom Shaders**: Specialized depth computation for specific sensors

### LiDAR Simulation in Unity

LiDAR (Light Detection and Ranging) simulation in Unity involves generating realistic point cloud data that matches the characteristics of real LiDAR sensors. This is achieved through:

**Raycasting Approach**: Unity can simulate LiDAR by casting rays from the sensor origin in the pattern of the real LiDAR, measuring distances to surfaces.

**Point Cloud Generation**: Converting ray intersection data into point cloud format compatible with robotics frameworks.

**Noise Modeling**: Adding realistic noise patterns that match real LiDAR sensors, including measurement uncertainty and dropouts.

**Intensity Simulation**: Modeling the intensity return values based on surface properties and lighting conditions.

### Advanced Sensor Simulation Techniques

Unity provides several advanced techniques for realistic sensor simulation:

**Custom Render Textures**: Creating specialized render targets for different sensor modalities
**Compute Shaders**: High-performance computation for sensor data processing
**Shader Graph**: Visual programming for custom sensor simulation effects
**Scriptable Render Pipeline**: Custom rendering pipelines for specialized sensor simulation

## Human-Robot Interaction in Unity

### Character Animation and Control

Unity's animation system provides sophisticated tools for humanoid robot control and interaction:

**Animation Controller**: State machines that manage different robot behaviors
**Blend Trees**: Smooth transitions between different animation states
**Inverse Kinematics**: Automated limb positioning for natural movement
**Animation Rigging**: Advanced control over character joint positions

For humanoid robotics, these tools enable:
- **Natural Movement**: Smooth, human-like motion patterns
- **Interactive Behaviors**: Responsive actions to environmental stimuli
- **Gesture Control**: Complex hand and body gestures
- **Emotional Expression**: Subtle movements that convey robot state

### Interaction Systems

Unity provides several systems for implementing human-robot interaction:

**Unity Input System**: Advanced input handling for user interaction
**XR Interaction Toolkit**: Tools for VR/AR interaction scenarios
**UI Systems**: Interfaces for human-robot communication
**Physics-Based Interaction**: Realistic object manipulation and physics response

### Behavior Trees and AI

Unity's AI capabilities can be leveraged for sophisticated robot behavior:

**NavMesh System**: Pathfinding and navigation in complex environments
**Behavior Trees**: Hierarchical task planning and execution
**Finite State Machines**: Simple behavior switching
**ML-Agents**: Machine learning for behavior optimization

## Integration with Robotics Frameworks

### ROS/ROS 2 Integration

Unity can be integrated with ROS/ROS 2 through several approaches:

**Unity Robotics Hub**: Official Unity package providing ROS connectivity
**ROS TCP Connector**: Network-based communication between Unity and ROS
**Custom Bridge Solutions**: Specialized integration for specific applications

The integration typically involves:

- **Message Publishing**: Unity publishing sensor data and robot state to ROS topics
- **Command Subscribing**: Unity receiving control commands from ROS
- **Transform Broadcasting**: Unity providing coordinate transforms for spatial reasoning
- **Service Integration**: Unity providing ROS services for robot control

### Unity ML-Agents for Robot Learning

Unity ML-Agents provides a platform for training intelligent agents using reinforcement learning:

**Environment Design**: Creating training environments with appropriate rewards and challenges
**Agent Definition**: Defining robot agents with observations, actions, and rewards
**Training Process**: Using Unity as a training environment for robot behaviors
**Model Deployment**: Deploying trained models to real robots

ML-Agents is particularly valuable for humanoid robotics because it allows for safe, rapid training of complex behaviors in simulation before deployment on physical robots.

### Isaac Sim Integration

NVIDIA Isaac Sim provides specialized tools for robotics simulation within Unity:

**Photorealistic Sensor Simulation**: Advanced camera and sensor simulation
**Synthetic Data Generation**: Tools for creating training datasets
**Robot Simulation**: Specialized tools for robot physics and control
**AI Training**: Integration with NVIDIA's AI training platforms

## Advanced Unity Features for Robotics

### Custom Shaders for Sensor Simulation

Unity's shader system allows for custom sensor simulation:

**Surface Shaders**: High-level shaders for complex lighting calculations
**Vertex and Fragment Shaders**: Low-level control over rendering
**Compute Shaders**: High-performance parallel computation
**Custom Sensor Shaders**: Specialized shaders for specific sensor types

Custom shaders can be used to:
- Simulate specific sensor characteristics
- Add realistic noise patterns
- Implement specialized sensor processing
- Optimize performance for real-time applications

### Performance Optimization

High-fidelity robotics simulation requires careful performance optimization:

**Occlusion Culling**: Reducing rendering load by not rendering occluded objects
**LOD Systems**: Using simplified models when detail isn't needed
**Multi-threading**: Utilizing multiple CPU cores for simulation
**GPU Acceleration**: Using GPU for physics and sensor computation
**Memory Management**: Efficient resource usage for long-running simulations

### Multi-Scene Management

Complex robotics applications may require multiple scenes:

**Scene Loading**: Dynamic loading of different environments
**Scene Streaming**: Loading large environments in chunks
**Scene Management**: Coordinating multiple simulation environments
**Resource Sharing**: Sharing assets between scenes efficiently

## Best Practices for Robotics Simulation in Unity

### Model Accuracy and Validation

Ensuring that Unity models accurately represent physical robots:

- **Mass Properties**: Accurate center of mass and moment of inertia
- **Joint Limits**: Correct position and velocity constraints
- **Sensor Placement**: Accurate positioning of virtual sensors
- **Material Properties**: Realistic surface properties for sensor simulation

### Performance Considerations

Balancing visual quality with computational performance:

- **Polygon Optimization**: Using appropriate polygon counts for real-time performance
- **Texture Resolution**: Balancing visual quality with memory usage
- **Lighting Complexity**: Using efficient lighting solutions
- **Physics Optimization**: Optimizing collision geometry and physics parameters

### Debugging and Visualization

Unity provides extensive tools for debugging robotic systems:

- **Scene View**: Visual debugging of robot state and environment
- **Game View**: Real-time visualization of robot perspective
- **Console**: Logging and error reporting
- **Profiler**: Performance analysis and optimization
- **Custom Gizmos**: Visual indicators for robot state and sensor data

## Case Studies and Applications

### Training Perception Systems

Unity's high-fidelity rendering makes it ideal for training computer vision systems:

- **Synthetic Data Generation**: Creating large datasets without physical data collection
- **Domain Randomization**: Varying visual properties to improve generalization
- **Edge Case Simulation**: Creating challenging scenarios for robust perception
- **Multi-Sensor Fusion**: Training systems that combine multiple sensor modalities

### Human-Robot Interaction Studies

Unity's interactive capabilities enable research into human-robot interaction:

- **Immersive Environments**: Creating realistic interaction scenarios
- **Behavioral Studies**: Studying human responses to robot behavior
- **Safety Validation**: Testing interaction safety in controlled environments
- **User Experience**: Designing intuitive human-robot interfaces

### Control Algorithm Development

Unity provides a safe environment for developing and testing control algorithms:

- **Balance Control**: Testing bipedal balance algorithms
- **Manipulation**: Developing dexterous manipulation skills
- **Navigation**: Testing path planning and obstacle avoidance
- **Learning**: Training adaptive control systems

## Conclusion

Unity represents a powerful platform for high-fidelity robotics simulation, particularly valuable for humanoid robotics applications that require photorealistic rendering, sophisticated sensor simulation, and human-robot interaction studies. Its combination of visual quality, flexible development environment, and integration capabilities makes it an essential tool in the modern robotics development pipeline.

While Unity may require more computational resources than traditional simulators, its ability to generate realistic synthetic data and create immersive environments provides unique advantages for developing advanced robotic systems. As robotics continues to advance, platforms like Unity will play an increasingly important role in bridging the gap between simulation and reality.

The next chapter will explore Isaac Sim, NVIDIA's specialized platform for robotics simulation that builds upon Unity's capabilities with additional tools specifically designed for robotics and AI applications.

---

## References

[1] Unity Technologies. (2023). Unity Robotics Hub Documentation. Retrieved from https://github.com/Unity-Technologies/Unity-Robotics-Hub

[2] NVIDIA. (2023). Isaac Sim Documentation. Retrieved from https://docs.nvidia.com/isaac/isaac_sim/

[3] Juliani, A., Berges, V. P., Vckay, E., Gao, Y., Henry, H., Mattar, M., & Lange, D. (2018). Unity: A general platform for intelligent agents. *arXiv preprint arXiv:1809.02627*.

[4] ROS.org. (2023). Unity Integration Tutorials. Retrieved from http://wiki.ros.org/Unity

[5] Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics*. Springer.

## Code Examples

### Example 1: Unity Robot Controller Script
```csharp
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class HumanoidRobotController : MonoBehaviour
{
    [Header("ROS Connection")]
    public string robotNamespace = "/humanoid_robot";

    [Header("Joint Configuration")]
    public List<ArticulationBody> joints;
    public List<string> jointNames;

    [Header("Sensor Configuration")]
    public Camera mainCamera;
    public Transform imuTransform;

    private ROSConnection ros;
    private Dictionary<string, float> targetJointPositions = new Dictionary<string, float>();
    private Dictionary<string, float> currentJointPositions = new Dictionary<string, float>();

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;

        // Subscribe to joint trajectory commands
        ros.Subscribe<trajectory_msgs.msg.JointTrajectory>(
            robotNamespace + "/joint_trajectory", JointTrajectoryCallback);

        // Initialize joint dictionaries
        InitializeJointDictionaries();

        // Start sensor publishers
        StartCoroutine(PublishJointStates());
        StartCoroutine(PublishCameraData());
        StartCoroutine(PublishIMUData());
    }

    void InitializeJointDictionaries()
    {
        for (int i = 0; i < joints.Count && i < jointNames.Count; i++)
        {
            targetJointPositions[jointNames[i]] = joints[i].jointPosition[0];
            currentJointPositions[jointNames[i]] = joints[i].jointPosition[0];
        }
    }

    void JointTrajectoryCallback(trajectory_msgs.msg.JointTrajectory trajectory)
    {
        if (trajectory.points.Count > 0)
        {
            var point = trajectory.points[0];

            for (int i = 0; i < trajectory.joint_names.Count; i++)
            {
                string jointName = trajectory.joint_names[i];
                if (targetJointPositions.ContainsKey(jointName))
                {
                    targetJointPositions[jointName] = (float)point.positions[i];
                }
            }
        }
    }

    void Update()
    {
        // Simple PD control to move joints toward target positions
        for (int i = 0; i < joints.Count && i < jointNames.Count; i++)
        {
            string jointName = jointNames[i];
            if (targetJointPositions.ContainsKey(jointName))
            {
                float targetPos = targetJointPositions[jointName];
                float currentPos = joints[i].jointPosition[0];

                // PD control
                float error = targetPos - currentPos;
                float force = 100f * error - 10f * joints[i].jointVelocity[0];

                joints[i].AddForce(force, ArticulationForceMode.Force);

                // Update current position dictionary
                currentJointPositions[jointName] = currentPos;
            }
        }
    }

    IEnumerator PublishJointStates()
    {
        while (true)
        {
            var jointStateMsg = new sensor_msgs.msg.JointState();
            jointStateMsg.header = new std_msgs.msg.Header();
            jointStateMsg.header.stamp = new builtin_interfaces.msg.Time();
            jointStateMsg.header.frame_id = "base_link";

            // Fill joint names and positions
            jointStateMsg.name = new string[currentJointPositions.Count];
            jointStateMsg.position = new double[currentJointPositions.Count];

            int index = 0;
            foreach (var kvp in currentJointPositions)
            {
                jointStateMsg.name[index] = kvp.Key;
                jointStateMsg.position[index] = kvp.Value;
                index++;
            }

            ros.Publish(robotNamespace + "/joint_states", jointStateMsg);
            yield return new WaitForSeconds(0.01f); // 100 Hz
        }
    }

    IEnumerator PublishCameraData()
    {
        while (true)
        {
            // Render camera image and convert to ROS message
            RenderTexture currentRT = RenderTexture.active;
            RenderTexture.active = mainCamera.targetTexture;

            mainCamera.Render();

            Texture2D imageTex = new Texture2D(mainCamera.targetTexture.width,
                                              mainCamera.targetTexture.height,
                                              TextureFormat.RGB24, false);
            imageTex.ReadPixels(new Rect(0, 0, mainCamera.targetTexture.width,
                                        mainCamera.targetTexture.height), 0, 0);
            imageTex.Apply();

            // Convert to byte array for ROS message
            byte[] imageData = imageTex.EncodeToPNG();

            // Create and publish image message
            var imageMsg = new sensor_msgs.msg.Image();
            imageMsg.header = new std_msgs.msg.Header();
            imageMsg.header.stamp = new builtin_interfaces.msg.Time();
            imageMsg.header.frame_id = "camera_optical_frame";
            imageMsg.height = (uint)mainCamera.targetTexture.height;
            imageMsg.width = (uint)mainCamera.targetTexture.width;
            imageMsg.encoding = "rgb8";
            imageMsg.is_bigendian = 0;
            imageMsg.step = (uint)(3 * mainCamera.targetTexture.width); // 3 bytes per pixel
            imageMsg.data = imageData;

            ros.Publish(robotNamespace + "/camera/image_raw", imageMsg);

            // Clean up
            RenderTexture.active = currentRT;
            Destroy(imageTex);

            yield return new WaitForSeconds(0.033f); // 30 Hz
        }
    }

    IEnumerator PublishIMUData()
    {
        while (true)
        {
            var imuMsg = new sensor_msgs.msg.Imu();
            imuMsg.header = new std_msgs.msg.Header();
            imuMsg.header.stamp = new builtin_interfaces.msg.Time();
            imuMsg.header.frame_id = "imu_link";

            // Get orientation from transform
            Quaternion rot = imuTransform.rotation;
            imuMsg.orientation.x = rot.x;
            imuMsg.orientation.y = rot.y;
            imuMsg.orientation.z = rot.z;
            imuMsg.orientation.w = rot.w;

            // Get angular velocity (simplified)
            imuMsg.angular_velocity.x = Random.Range(-0.1f, 0.1f);
            imuMsg.angular_velocity.y = Random.Range(-0.1f, 0.1f);
            imuMsg.angular_velocity.z = Random.Range(-0.1f, 0.1f);

            // Get linear acceleration (simplified)
            imuMsg.linear_acceleration.x = Random.Range(-0.5f, 0.5f);
            imuMsg.linear_acceleration.y = Random.Range(-0.5f, 0.5f);
            imuMsg.linear_acceleration.z = Random.Range(-10.0f, -8.0f); // Gravity

            ros.Publish(robotNamespace + "/imu/data", imuMsg);
            yield return new WaitForSeconds(0.01f); // 100 Hz
        }
    }
}
```

### Example 2: Unity LiDAR Sensor Simulation
```csharp
using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class UnityLidarSensor : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public float minAngle = -90f;      // Degrees
    public float maxAngle = 90f;       // Degrees
    public int numRays = 360;          // Number of rays
    public float maxRange = 20f;       // Maximum range in meters
    public string frameId = "lidar_link";

    [Header("Noise Configuration")]
    public float rangeNoiseStd = 0.01f; // Standard deviation of range noise

    private ROSConnection ros;
    private string robotNamespace = "/humanoid_robot";

    void Start()
    {
        ros = ROSConnection.instance;
        StartCoroutine(PublishLidarData());
    }

    IEnumerator PublishLidarData()
    {
        while (true)
        {
            var laserMsg = new sensor_msgs.msg.LaserScan();
            laserMsg.header = new std_msgs.msg.Header();
            laserMsg.header.stamp = new builtin_interfaces.msg.Time();
            laserMsg.header.frame_id = frameId;

            laserMsg.angle_min = minAngle * Mathf.Deg2Rad;
            laserMsg.angle_max = maxAngle * Mathf.Deg2Rad;
            laserMsg.angle_increment = (maxAngle - minAngle) * Mathf.Deg2Rad / numRays;
            laserMsg.time_increment = 0;
            laserMsg.scan_time = 0.1f; // 10 Hz
            laserMsg.range_min = 0.1f;
            laserMsg.range_max = maxRange;

            // Perform raycasting to get distance measurements
            List<float> ranges = new List<float>();

            for (int i = 0; i < numRays; i++)
            {
                float angle = minAngle + (maxAngle - minAngle) * i / numRays;
                Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, maxRange))
                {
                    float distance = hit.distance;
                    // Add noise to simulate real sensor
                    distance += Random.Range(-rangeNoiseStd, rangeNoiseStd);
                    ranges.Add(distance);
                }
                else
                {
                    ranges.Add(float.PositiveInfinity); // No obstacle detected
                }
            }

            laserMsg.ranges = new float[ranges.Count];
            for (int i = 0; i < ranges.Count; i++)
            {
                laserMsg.ranges[i] = ranges[i];
            }

            // Intensities (optional)
            laserMsg.intensities = new float[ranges.Count];
            for (int i = 0; i < ranges.Count; i++)
            {
                laserMsg.intensities[i] = 100f; // Simulated intensity
            }

            ros.Publish(robotNamespace + "/scan", laserMsg);
            yield return new WaitForSeconds(0.1f); // 10 Hz
        }
    }

    // Visualization of LiDAR rays (for debugging)
    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;

        Gizmos.color = Color.red;

        for (int i = 0; i < numRays; i++)
        {
            float angle = minAngle + (maxAngle - minAngle) * i / numRays;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                Gizmos.DrawLine(transform.position, hit.point);
            }
            else
            {
                Gizmos.DrawLine(transform.position, transform.position + direction * maxRange);
            }
        }
    }
}
```

### Example 3: Unity Scene Setup for Robotics Simulation
```csharp
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;

public class RoboticsSceneSetup : MonoBehaviour
{
    [Header("Robot Configuration")]
    public GameObject humanoidRobotPrefab;
    public Transform spawnPoint;

    [Header("Environment Configuration")]
    public GameObject[] environmentPrefabs;
    public Light mainLight;

    [Header("ROS Configuration")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    private ROSConnection ros;

    void Start()
    {
        // Setup ROS connection
        SetupROSConnection();

        // Spawn robot
        SpawnRobot();

        // Setup environment
        SetupEnvironment();

        // Configure lighting
        ConfigureLighting();

        // Initialize sensor systems
        InitializeSensorSystems();
    }

    void SetupROSConnection()
    {
        ros = ROSConnection.instance;
        ros.Initialize(rosIPAddress, rosPort);

        Debug.Log($"ROS Connection initialized to {rosIPAddress}:{rosPort}");
    }

    void SpawnRobot()
    {
        if (humanoidRobotPrefab != null && spawnPoint != null)
        {
            GameObject robot = Instantiate(humanoidRobotPrefab, spawnPoint.position, spawnPoint.rotation);
            robot.name = "HumanoidRobot";

            // Initialize robot components
            var controller = robot.GetComponent<HumanoidRobotController>();
            if (controller != null)
            {
                controller.robotNamespace = "/humanoid_robot";
            }

            Debug.Log("Robot spawned successfully");
        }
    }

    void SetupEnvironment()
    {
        // Randomly place environment objects
        foreach (var envPrefab in environmentPrefabs)
        {
            if (envPrefab != null)
            {
                // Place at random positions within bounds
                Vector3 randomPos = new Vector3(
                    Random.Range(-10f, 10f),
                    0f,
                    Random.Range(-10f, 10f)
                );

                GameObject envObj = Instantiate(envPrefab, randomPos, Quaternion.identity);
                envObj.name = envPrefab.name;
            }
        }

        Debug.Log("Environment setup completed");
    }

    void ConfigureLighting()
    {
        if (mainLight != null)
        {
            // Configure realistic lighting
            mainLight.type = LightType.Directional;
            mainLight.intensity = 1.0f;
            mainLight.color = Color.white;
            mainLight.shadows = LightShadows.Soft;
            mainLight.shadowStrength = 0.8f;

            // Set light direction (typical outdoor lighting)
            mainLight.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        }

        // Add ambient lighting
        RenderSettings.ambientLight = new Color(0.2f, 0.2f, 0.2f);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
    }

    void InitializeSensorSystems()
    {
        // Initialize any sensor-specific systems
        // This could include setting up camera calibration, sensor noise parameters, etc.

        Debug.Log("Sensor systems initialized");
    }

    // Scene reset functionality
    public void ResetScene()
    {
        // Destroy all spawned objects except the setup controller
        GameObject[] objects = GameObject.FindGameObjectsWithTag("Robot");
        foreach (GameObject obj in objects)
        {
            if (obj != this.gameObject)
            {
                DestroyImmediate(obj);
            }
        }

        // Re-spawn robot and re-setup environment
        SpawnRobot();
        SetupEnvironment();
    }

    void OnApplicationQuit()
    {
        // Clean up ROS connection
        if (ros != null)
        {
            ros.Close();
        }
    }
}
```

### Example 4: ROS Launch File for Unity Integration
```xml
<launch>
  <!-- Unity Robot Simulation -->
  <arg name="unity_ip" default="127.0.0.1"/>
  <arg name="unity_port" default="10000"/>
  <arg name="robot_namespace" default="/humanoid_robot"/>

  <!-- Unity ROS Bridge -->
  <node pkg="unity_robotics_demo" exec="unity_ros_bridge" name="unity_ros_bridge" output="screen">
    <param name="unity_ip" value="$(var unity_ip)"/>
    <param name="unity_port" value="$(var unity_port)"/>
    <param name="robot_namespace" value="$(var robot_namespace)"/>
  </node>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
    <param name="rate" value="50"/>
  </node>

  <!-- TF publisher for Unity coordinate frame -->
  <node pkg="tf2_ros" exec="static_transform_publisher" name="unity_world_tf">
    <param name="x" value="0"/>
    <param name="y" value="0"/>
    <param name="z" value="0"/>
    <param name="qx" value="0"/>
    <param name="qy" value="0"/>
    <param name="qz" value="0"/>
    <param name="qw" value="1"/>
    <param name="frame_id" value="world"/>
    <param name="child_frame_id" value="unity_origin"/>
  </node>

  <!-- Point cloud processing for Unity LiDAR data -->
  <node pkg="pointcloud_to_laserscan" exec="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
    <param name="target_frame" value="lidar_link"/>
    <param name="source_frame" value="lidar_link"/>
    <param name="min_height" value="-0.2"/>
    <param name="max_height" value="0.2"/>
    <param name="angle_min" value="-1.5708"/> <!-- -90 degrees -->
    <param name="angle_max" value="1.5708"/>  <!-- 90 degrees -->
    <param name="angle_increment" value="0.0087"/> <!-- 0.5 degrees -->
    <param name="scan_time" value="0.1"/>
    <param name="range_min" value="0.1"/>
    <param name="range_max" value="20.0"/>
    <param name="use_inf" value="true"/>
    <param name="inf_epsilon" value="1.0"/>
  </node>

  <!-- Robot controller -->
  <node pkg="humanoid_control" exec="unity_humanoid_controller" name="unity_humanoid_controller" output="screen">
    <param name="control_frequency" value="100"/>
    <param name="unity_simulation" value="true"/>
  </node>

  <!-- RViz for visualization -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share humanoid_description)/rviz/unity_robot.rviz"/>
</launch>
```

### Example 5: Unity Custom Shader for Sensor Simulation
```hlsl
// Custom shader for simulating depth sensor with noise
Shader "Robotics/DepthSensor"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _DepthScale ("Depth Scale", Range(0.1, 10.0)) = 1.0
        _NoiseIntensity ("Noise Intensity", Range(0.0, 0.1)) = 0.01
        _MinDistance ("Min Distance", Float) = 0.1
        _MaxDistance ("Max Distance", Float) = 20.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float depth : TEXCOORD1;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float _DepthScale;
            float _NoiseIntensity;
            float _MinDistance;
            float _MaxDistance;

            // Simple noise function
            float noise(float2 seed)
            {
                return frac(sin(dot(seed, float2(12.9898, 78.233))) * 43758.5453);
            }

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);

                // Calculate depth from camera
                float4 worldPos = mul(unity_ObjectToWorld, v.vertex);
                o.depth = distance(_WorldSpaceCameraPos, worldPos.xyz);

                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // Sample the main texture
                fixed4 col = tex2D(_MainTex, i.uv);

                // Calculate normalized depth
                float normalizedDepth = saturate((i.depth - _MinDistance) / (_MaxDistance - _MinDistance));

                // Add noise to simulate sensor imperfections
                float noiseValue = noise(i.uv * _Time.x) * _NoiseIntensity;
                normalizedDepth += noiseValue;

                // Clamp to valid range
                normalizedDepth = clamp(normalizedDepth, 0.0, 1.0);

                // Output depth as grayscale for depth visualization
                return fixed4(normalizedDepth, normalizedDepth, normalizedDepth, 1.0);
            }
            ENDCG
        }
    }
}
```