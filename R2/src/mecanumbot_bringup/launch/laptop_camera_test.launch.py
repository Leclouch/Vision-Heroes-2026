import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanumbot_bringup')
    
    # Use the same config but we can override topics
    config_file = os.path.join(pkg_share, 'config', 'apriltag_config.yaml')

    # Laptop Camera Node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='laptop_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
        }],
        output='screen'
    )

    # AprilTag Node remapped to laptop camera
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_laptop',
        parameters=[config_file],
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        apriltag_node
    ])
