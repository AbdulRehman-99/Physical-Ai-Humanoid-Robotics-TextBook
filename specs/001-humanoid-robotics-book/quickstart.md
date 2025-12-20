# Quickstart Guide: Physical AI & Humanoid Robotics Technical Book

## Overview
This quickstart guide provides the essential steps to get started with the Physical AI & Humanoid Robotics technical book, including environment setup, basic examples, and first steps for each major technology covered.

## Prerequisites

### Hardware Requirements
- **CPU**: 8-core processor (Intel Core i7 9th Gen+ or AMD Ryzen 7)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: Dedicated graphics card with 4GB+ VRAM (NVIDIA RTX 3070 or equivalent)
- **Storage**: 500GB SSD minimum
- **OS**: Ubuntu 22.04 LTS (primary recommended)

### Software Requirements
- **Python**: 3.10+
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Harmonic
- **Unity**: 2023 LTS
- **Isaac ROS**: Latest stable release
- **Docusaurus**: v3.9

## Environment Setup

### 1. Install Ubuntu 22.04 LTS
Download and install Ubuntu 22.04 LTS from the official website. For dual-boot systems, ensure at least 50GB of free space.

### 2. Install Python 3.10+
```bash
sudo apt update
sudo apt install python3.10 python3.10-venv python3.10-dev
```

### 3. Install ROS 2 Humble Hawksbill
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install ros-dev-tools

# Setup environment
source /opt/ros/humble/setup.bash
```

### 4. Install Gazebo Harmonic
```bash
# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-*
```

### 5. Install Isaac ROS
Follow the official Isaac ROS installation guide for Ubuntu 22.04:
```bash
# Add NVIDIA package repositories
curl -sL https://nvidia.github.io/nvs/cfg/nvs-jenkins.pub | sudo apt-key add -
curl -sL https://nvidia.github.io/isaac_ros/setup_jenkins.deb.sh | sudo bash
sudo apt-get update

# Install Isaac ROS packages
sudo apt-get install IsaacROSCommon isaac_ros_common-dev
```

### 6. Install Docusaurus
```bash
# Install Node.js (v18.0 or higher)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install Docusaurus globally
npm init docusaurus@latest my-website classic
```

## Getting Started with Examples

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book
```

### 2. Set up Python Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
```

### 3. Basic ROS 2 Example: Publisher/Subscriber
Navigate to the first module examples:
```bash
cd src/ros2-nodes/python
python3 publisher.py
# In another terminal:
python3 subscriber.py
```

### 4. Basic Gazebo Simulation
Launch the humanoid robot simulation:
```bash
source /opt/ros/humble/setup.bash
cd ~/physical-ai-humanoid-robotics-book/src/simulation-configs/gazebo-worlds
ros2 launch humanoid_simulation.launch.py
```

### 5. Build and Serve the Book
```bash
cd docusaurus
npm install
npm run build
npm run serve
```

## Chapter Structure Overview

The book is organized into 4 modules with 2 chapters each (8 chapters total), with each chapter containing at least 2500 words following the pattern:

1. **Concept explanation** - Theoretical background
2. **Diagram/code** - Visual representations and code examples
3. **Applied example** - Practical implementation
4. **References** - Citations in IEEE format

### Module 1: ROS 2 (Chapters 1-2)
- Chapter 1: Foundations + Humanoid Robotics Basics
- Chapter 2: ROS 2: The Robotic Nervous System

### Module 2: Simulation (Chapters 3-4)
- Chapter 3: Gazebo: Digital Twin & Physics
- Chapter 4: Unity: High-Fidelity Interaction & Sensors

### Module 3: NVIDIA Isaac (Chapters 5-6)
- Chapter 5: Isaac Sim: Perception, Synthetic Data & Environments
- Chapter 6: Isaac ROS: VSLAM, Localization & Nav2 for Humanoids

### Module 4: Advanced Integration (Chapters 7-8)
- Chapter 7: Whisper, LLM/VLA Planning
- Chapter 8: Capstone: Humanoid Autonomy

## Key URDF Model Setup

The book uses 28-32 DOF humanoid models similar to HRP-4. To load the model:

```bash
# Launch with Gazebo
roslaunch gazebo_ros empty_world.launch
rosrun gazebo_ros spawn_model -file `rospack find humanoid_description`/urdf/humanoid.urdf -urdf -model humanoid

# Or with ROS 2
ros2 run gazebo_ros spawn_entity.py -file ~/physical-ai-humanoid-robotics-book/src/urdf-models/humanoid-32dof.urdf -entity humanoid
```

## Running Simulation Examples

### Gazebo Example
```bash
cd ~/physical-ai-humanoid-robotics-book/src/simulation-configs/gazebo-worlds
ros2 launch humanoid_gazebo.launch.py
```

### Unity Example
Unity examples are located in the `simulation-configs/unity-scenes` directory. Open Unity Hub, import the project, and run the scenes from the Assets/Scenes directory.

### Isaac Sim Example
```bash
# Launch Isaac Sim
isaac-sim python3 scripts/humanoid_perception.py
```

## Troubleshooting Common Issues

### Python Virtual Environment Issues
If you encounter issues with ROS 2 and Python packages:
```bash
# Always source ROS 2 before activating virtual environment
source /opt/ros/humble/setup.bash
python3 -m venv venv
source venv/bin/activate
# Install packages after sourcing ROS 2
```

### Gazebo Performance Issues
- Ensure GPU drivers are properly installed
- Reduce physics update rate in world files
- Close other GPU-intensive applications

### Isaac Sim Installation Issues
- Verify NVIDIA GPU compatibility
- Ensure CUDA drivers are properly installed
- Check that Isaac ROS packages are correctly installed

## Next Steps

1. Complete the setup verification by running the basic examples
2. Review the first chapter on humanoid robotics fundamentals
3. Follow along with the ROS 2 basics in Chapter 2
4. Explore the simulation environments in Modules 2 and 3
5. Integrate everything in the advanced VLA systems in Module 4

## Getting Help

- Check the book's reference materials and glossary
- Review the troubleshooting section in the reference documentation
- Consult the official documentation for ROS 2, Gazebo, Unity, and Isaac Sim
- Join the community forums for the book (if available)
