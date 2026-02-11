import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mecanum_bot_description_path = get_package_share_directory("mecanumbot_description")

    # Define paths
    xacro_file = os.path.join(
        mecanum_bot_description_path, "urdf", "mecanum_robot.xacro"
    )
    urdf_file = "/tmp/mecanumbot.urdf"

    # Launch argument for world file
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="empty.world",
        description="World file to load (empty.world atau optimized.world)",
    )
    world_file_config = LaunchConfiguration("world")
    world_file = [mecanum_bot_description_path, "/world/", world_file_config]

    # Set IGN_GAZEBO_RESOURCE_PATH to include the parent of mecanumbot_description
    # This allows resolving model://mecanumbot_description if needed, though we use file:// for meshes
    ign_resource_path = os.path.dirname(mecanum_bot_description_path)
    set_ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=ign_resource_path
    )
    set_ign_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH", value="/opt/ros/humble/lib"
    )

    # Launch Arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # 1. Generate URDF
    generate_urdf = ExecuteProcess(
        cmd=["xacro", xacro_file, "-o", urdf_file], output="screen"
    )

    # 2. Replace package:// with absolute file:// paths for Ignition
    # This is necessary because ign gazebo might not resolve package:// without ros_ign_sim
    replace_package_uri = ExecuteProcess(
        cmd=[
            "sed",
            "-i",
            f"s|package://mecanumbot_description|file://{mecanum_bot_description_path}|g",
            urdf_file,
        ],
        output="screen",
    )

    # 3. Launch Ignition Gazebo
    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_file], output="screen"
    )

    # 4. Spawn Robot
    # We spawn from the generated URDF file
    spawn_robot = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            "/world/empty_world/create",
            "--reqtype",
            "ignition.msgs.EntityFactory",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f'sdf_filename: "{urdf_file}", name: "mecanum_robot",pose:{{position:{{x: 0.0, y: 0.0, z: 1.0}}}}',
        ],
        output="screen",
    )

    # 5. Robot State Publisher
    # We need to read the URDF file content for the publisher
    # Since the file is generated at runtime, we can't easily pass it as parameter at launch construction time
    # However, RSP supports reading from topic if "robot_description" is published.
    # But usually we pass "robot_description" parameter.
    # For simplicity, we will run xacro again using Command substitution for RSP,
    # as RSP needs it at startup and doesn't wait for file generation.
    # The double xacro run is acceptable for this workaround.

    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterValue

    robot_description_content = Command(["xacro ", xacro_file])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            robot_description,
        ],
    )

    # 6. Bridge
    # Bridge /clock and other topics
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # Add other needed bridges here if not handled by plugins
        ],
        output="screen",
    )

    # 7. Spawners
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_mecanumbot_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanumbot_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # 8. Robot Mover Node (publishes cmd_vel to move robot)
    # Delay 8 seconds to ensure controller is fully active
    # rbot_mover = TimerAction(
    #     period=8.0,
    #     actions=[
    #         Node(
    #             package="mecanumbot_bringup",
    #             executable="rbot_mover.py",
    #             output="screen",
    #             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    #         )
    #     ],
    # )

    # Event Handlers to ensure order
    # Generate -> Sed -> Spawn

    sed_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_urdf, on_exit=[replace_package_uri]
        )
    )

    spawn_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=replace_package_uri, on_exit=[spawn_robot]
        )
    )

    return LaunchDescription(
        [
            set_ign_resource_path,
            set_ign_plugin_path,
            use_sim_time,
            declare_world,
            generate_urdf,
            sed_event,
            spawn_event,
            ign_gazebo,
            robot_state_publisher,
            bridge,
            spawn_joint_state_broadcaster,
            spawn_mecanumbot_drive_controller,
            # rbot_mover,
        ]
    )
