import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup_prefix = get_package_prefix('mecanumbot_bringup')
    joy_teleop_script = os.path.join(pkg_bringup_prefix, 'lib', 'mecanumbot_bringup', 'joy_teleop.py')

    return LaunchDescription([
        # Joy Node (reads joystick input)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        # Custom Joy Teleop Node (converts joy to cmd_vel)
        # Using python3.10 explicitly to bypass partition permission issues
        # package is omitted to allow running system executable python3.10
        Node(
            executable='/usr/bin/python3.10',
            name='joy_teleop',
            output='screen',
            arguments=[joy_teleop_script],
            parameters=[{
                'axis_linear_x': 1,
                'axis_linear_y': 0,
                'axis_angular': 3,
                'scale_linear': 0.5,
                'scale_angular': 1.0,
                'require_enable_button': False
            }]
        )
    ])
