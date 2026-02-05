# Mecanum Bot ROS2

A ROS2-based control system for a Mecanum wheel robot with Gazebo simulation support.

## Project Structure

- **mecanumbot_bringup** - Launch files for starting the robot system
- **mecanumbot_description** - Robot URDF models, meshes, and Gazebo world files
- **mecanumbot_controller** - Custom Mecanum drive controller plugin
- **mecanumbot_hardware** - Hardware interface for motor control
- **mecanumbot_control** - Controller manager configuration
- **mecanumbot_teleop** - Joystick teleoperation node

## Prerequisites

- ROS2 Humble or later
- Gazebo (Ignition)
- colcon build tools

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-* ros-humble-controller-manager
```

## Setup

1. **Clone and navigate to workspace:**
```bash
cd ~/mecanum_ws-main
```

2. **Install dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace:**
```bash
colcon build
```

4. **Source the setup script:**
```bash
source install/setup.bash
```

## Usage

### Launch Robot in Gazebo Simulation

```bash
ros2 launch mecanumbot_bringup gazebo.launch.py
```

This will:
- Start Gazebo with the empty world
- Spawn the Mecanum robot at position (3, 3, 1.0)
- Initialize the joint state broadcaster
- Initialize the Mecanum drive controller

### View Robot State with RViz

```bash
ros2 launch mecanumbot_bringup rviz2.py
```

### Teleoperation with Joystick

```bash
ros2 launch mecanumbot_bringup joy_teleop.launch.py
```

## Features

- **Hardware Abstraction** - Clean interface for motor control via ROS2 hardware_interface
- **Controller Stack** - Uses ROS2 control framework for modular control
- **Simulation Ready** - Full Gazebo integration with proper resource URIs
- **Teleoperation** - Joystick-based remote control

## Notes

- Robot spawns at (x=3, y=3, z=1.0) in the Gazebo world
- Uses ODE physics engine with 0.01s time step
- All resource URIs use `package://` scheme for cross-platform compatibility

## License

Apache 2.0
