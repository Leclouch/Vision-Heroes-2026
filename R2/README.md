# Mecanum Bot ROS2

A ROS2-based control system for a Mecanum wheel robot with Gazebo simulation support.

## Project Structure

- **mecanumbot_bringup** - Launch files for starting the robot system
- **mecanumbot_description** - Robot URDF models, meshes, and Gazebo world files
- **mecanumbot_controller** - Custom Mecanum drive controller plugin
- **mecanumbot_hardware** - Hardware interface for motor control
- **mecanumbot_control** - Controller manager configuration
- **mecanumbot_teleop** - Joystick teleoperation node
- **mecanumbot_apriltag** - AprilTag distance calculation node
- **mecanumbot_tag_follower** - Node for following tags at a specific distance

## Prerequisites

...

- [apriltag_msgs](https://github.com/christianrauch/apriltag_msgs)
- [apriltag_ros](https://github.com/christianrauch/apriltag_ros)

## Usage

### 1. Launch Robot in Gazebo Simulation

```bash
ros2 launch mecanumbot_bringup gazebo.launch.py
```

### 2. Start AprilTag Detection

```bash
ros2 launch mecanumbot_bringup apriltag.launch.py
```

### 3. Run Distance Calculator

```bash
ros2 run mecanumbot_apriltag apriltag_distance_node
```

### 4. Run Tag Follower

```bash
ros2 run mecanumbot_tag_follower tag_follower_node --ros-args -p target_distance:=1.0
```

## Features

- **AprilTag Tracking** - Accurate distance calculation using TF transforms.
- **Dynamic Following** - Robot maintains a target distance using a proportional controller.
- **Hardware Abstraction** - Clean interface for motor control via ROS2 hardware_interface.
- **Simulation Ready** - Full Gazebo integration with front-facing camera.

## Notes

- **Camera Calibration**: If the distance is inaccurate, tune the `size` parameter in `apriltag_config.yaml` to match the **black square** portion of your tag.
- **Front Camera**: The camera is mounted on the front (`+0.12m`). The robot spawns facing the tag at Yaw 0.

## Distrobox (wafdan)

```bash
distrobox enter ros2-humble
cd ~/R2_Heroes_ws/R2
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch mecanumbot_bringup gazebo.launch.py
```

## License

Apache 2.0
