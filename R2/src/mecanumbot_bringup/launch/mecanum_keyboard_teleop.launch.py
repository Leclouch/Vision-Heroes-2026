import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_bringup_prefix = get_package_prefix("mecanumbot_bringup")
    keyboard_teleop_script = os.path.join(
        pkg_bringup_prefix, "lib", "mecanumbot_bringup", "mecanum_keyboard_teleop.py"
    )

    return LaunchDescription(
        [
            # Mecanum Keyboard Teleop Node
            # Running in separate terminal for keyboard input
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--title=Mecanum Keyboard Teleop",
                    "--",
                    "python3",
                    keyboard_teleop_script,
                ],
                output="screen",
            )
        ]
    )
