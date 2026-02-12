import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    mecanum_bot_description_path = get_package_share_directory("mecanumbot_description")
    mecanum_bot_description_prefix = get_package_prefix("mecanumbot_description")
    
    # Get the launch configuration
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Get the robot description
    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(
                mecanum_bot_description_path,
                "urdf",
                "mecanumbot.urdf.xacro",
            ),
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    # Gazebo Sim
    world_file = os.path.join(
        mecanum_bot_description_path,
        'world',
        'empty.world'
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanumbot',
            '-allow_renaming', 'true',
            '-x', '2',
            '-y', '5',
            '-z', '2.0',
            '-Y', '3.14159'
        ],
        output='screen'
    )

    # Spawn controllers
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_mecanumbot_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanumbot_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Resource paths
    current_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    current_ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    
    extra_resource_paths = [
        os.path.join(mecanum_bot_description_prefix, 'share'),
        os.path.join(mecanum_bot_description_path, 'models'),
        os.path.join(mecanum_bot_description_path, 'meshes'),
    ]
    
    new_gz_resource_path = ":".join(extra_resource_paths) + (":" + current_gz_resource_path if current_gz_resource_path else "")
    new_ign_resource_path = ":".join(extra_resource_paths) + (":" + current_ign_resource_path if current_ign_resource_path else "")

    return LaunchDescription(
        [
            SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=new_gz_resource_path),
            SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=new_ign_resource_path),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            gz_sim,
            robot_state_publisher,
            spawn_robot,
            spawn_joint_state_broadcaster,
            spawn_mecanumbot_drive_controller,
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/world/empty_world/model/mecanumbot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/world/empty_world/model/mecanumbot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
                output='screen'
            ),
        ]
    )