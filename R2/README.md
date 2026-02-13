# Mecanum Bot ROS 2

A modular ROS 2-based control system for a Mecanum wheel robot featuring AprilTag tracking and autonomous following using Gazebo simulation.

## 🏗 System Architecture

The project follows a decoupled architecture where detection and control are separated:

1. **Distance & Yaw Calculator** (`mecanumbot_apriltag`):
    * Processes raw AprilTag detections.
    * Calculates Euclidean distance and Yaw relative to the camera.
    * Publishes data to the `/tag_info` topic (`PointStamped`).
2. **Tag Follower** (`mecanumbot_tag_follower`):
    * Subscribes to `/tag_info`.
    * Implements the following logic: **Move forward if distance > 1.0m, else stop.**

## 📥 Installation

### 1. Prerequisites

Ensure you have **ROS 2 Humble** installed. If you are using **Distrobox**, refer to the `distrobox.sh` script for the environment setup.

### 2. Install Dependencies

Run the following commands to install the required ROS 2 packages and system dependencies:

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-apriltag-msgs ros-humble-apriltag-ros \
                 ros-humble-controller-manager ros-humble-hardware-interface \
                 ros-humble-tf2-ros ros-humble-xacro \
                 ros-humble-joint-state-broadcaster

# Install additional system dependencies via rosdep
cd ~/R2_Heroes_ws/R2
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### 3. Build the Workspace

```bash
colcon build
source install/setup.bash
```

## 🚀 Quick Start Guide

Follow these steps in separate terminals (ensure each terminal is sourced: `source install/setup.bash`).

### Step 1: Launch Simulation

Start the robot and the Gazebo world.

```bash
ros2 launch mecanumbot_bringup gazebo.launch.py
```

### Step 2: Start Tag Detection

Launches the camera processing pipeline for AprilTags.

```bash
ros2 launch mecanumbot_bringup apriltag.launch.py
```

### Step 3: Run the Data Calculator

Extracts the ID, Distance, and Yaw from detections.

```bash
ros2 run mecanumbot_apriltag apriltag_distance_node
```

### Step 4: Start the Follower

Makes the robot autonomously follow the tag.

```bash
ros2 run mecanumbot_tag_follower tag_follower_node
```

## 🛠 Advanced Usage

### Monitoring Data

You can monitor the live tag information (ID, Distance in meters, Yaw in radians) directly from the topic:

```bash
ros2 topic echo /tag_info
```

### Tuning Parameters

You can override the target distance or speed via CLI:

```bash
ros2 run mecanumbot_tag_follower tag_follower_node --ros-args -p target_distance:=1.5 -p linear_speed:=0.3
```

## 📝 Notes

* **Camera Placement**: The camera is mounted at `xyz="-0.12 0.0 0.1"` and rotated 180 degrees (`rpy="0 0 3.14"`) to align with the robot's forward movement.
* **Simulation Performance**: If you encounter graphics lag in Distrobox, ensure `export LIBGL_ALWAYS_SOFTWARE=1` is set in your environment.

## License

Apache 2.0
