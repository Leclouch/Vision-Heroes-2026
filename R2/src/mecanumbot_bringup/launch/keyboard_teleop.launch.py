import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup_prefix = get_package_prefix("mecanumbot_bringup")
    keyboard_teleop_script = os.path.join(
        pkg_bringup_prefix, "lib", "mecanumbot_bringup", "simple_teleop.py"
    )

    return LaunchDescription(
        [
            # Keyboard Teleop Node
            # Using python3.10 explicitly to bypass partition permission issues
            # Using gnome-terminal to provide terminal for keyboard input
            Node(
                executable="/usr/bin/python3.10",
                name="keyboard_teleop",
                output="screen",
                arguments=[keyboard_teleop_script],
                prefix=["gnome-terminal -- bash -c"],
            )
        ]
    )
