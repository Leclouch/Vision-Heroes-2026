import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_bringup_prefix = get_package_prefix("mecanumbot_bringup")
    keyboard_teleop_script = os.path.join(
        pkg_bringup_prefix, "lib", "mecanumbot_bringup", "keyboard_teleop.py"
    )

    # Launch argument for use_sim_time
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            # Keyboard Teleop Node
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
            ),
        ]
    )
