# Quickstart Guide: Humanoid Robotics & Embodied Intelligence Book

This guide provides a quick overview of how to set up your development environment to work with the code examples and exercises presented in this technical book.

## 1. System Requirements

Ensure your system meets the following recommended specifications for optimal performance with ROS 2, Gazebo, Unity, and Isaac Sim:

*   **Operating System**: Ubuntu 22.04 LTS (recommended primary OS)
*   **CPU**: Intel Core i7 (9th Gen+) or AMD Ryzen 7 (8 cores+ equivalent)
*   **RAM**: 32 GB minimum, 64 GB recommended
*   **GPU**: NVIDIA RTX 3070 minimum, RTX 4090 recommended, with at least 16 GB VRAM (critical for Isaac Sim)
*   **Storage**: 500 GB SSD or more

## 2. Software Installation

### 2.1. Ubuntu Setup

It is highly recommended to use Ubuntu 22.04 LTS. Follow the official Ubuntu installation guide if you don't already have it installed.

### 2.2. Python Environment

The code examples primarily use **Python 3.8-3.10**. It is recommended to use your system's Python 3 installation or a compatible virtual environment.

```bash
# Example: Check Python version
python3 --version
```

### 2.3. ROS 2 Installation

Install **ROS 2 Humble Hawksbill** (LTS) by following the official ROS 2 documentation for Ubuntu 22.04 LTS:
[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 2.4. Gazebo Simulation

Gazebo Fortress is the recommended simulator for ROS 2 Humble. It is typically installed as part of the ROS 2 installation. Verify its installation:

```bash
gazebo --version
```

### 2.5. Unity 3D (Optional for specific chapters)

For chapters involving Unity, download and install **Unity 2022 LTS** or **Unity 6.3 LTS** from the official Unity Hub:
[https://unity.com/download](https://unity.com/download)

### 2.6. NVIDIA Isaac Sim (Optional for specific chapters)

For chapters leveraging Isaac Sim, install the **latest stable release of NVIDIA Isaac Sim**. This typically requires an NVIDIA GPU and specific drivers. Refer to the official NVIDIA Isaac Sim documentation for detailed installation instructions:
[https://developer.nvidia.com/omniverse/isaac-sim/get-started](https://developer.nvidia.com/omniverse/isaac-sim/get-started)

## 3. Cloning the Code Examples

The code examples for this book are hosted in a Git repository. Clone the repository to your local machine:

```bash
git clone [REPOSITORY_URL_TO_BE_PROVIDED]
cd [REPOSITORY_NAME]
```

## 4. Running Examples

Each chapter will provide specific instructions on how to set up and run its corresponding code examples. This typically involves:

1.  Navigating to the chapter's example directory.
2.  Installing Python dependencies (e.g., `pip install -r requirements.txt`).
3.  Sourcing the ROS 2 environment.
4.  Executing ROS 2 nodes or Python scripts.

Follow the instructions within each chapter for detailed guidance.
