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
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mecanum_bot_description_path = get_package_share_directory("mecanumbot_description")

    # Define paths
    xacro_file = os.path.join(
        mecanum_bot_description_path, "urdf", "robots", "rosmaster_x3.urdf.xacro"
    )
    urdf_file = "/tmp/rosmaster_x3.urdf"

    # World file
    world_file = os.path.join(mecanum_bot_description_path, "world", "empty.world")

    # Set IGN_GAZEBO_RESOURCE_PATH
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

    # 1. Generate URDF with use_gazebo:=true
    generate_urdf = ExecuteProcess(
        cmd=[
            "xacro",
            xacro_file,
            "use_gazebo:=true",
            "robot_name:=rosmaster_x3",
            "-o",
            urdf_file,
        ],
        output="screen",
    )

    # 2. Replace package:// with absolute file:// paths for Ignition
    replace_package_uri = ExecuteProcess(
        cmd=[
            "sed",
            "-i",
            f"s|package://mecanumbot_description|file://{mecanum_bot_description_path}|g",
            urdf_file,
        ],
        output="screen",
    )

    # 3. Launch Ignition Gazebo (start immediately, will wait for robot)
    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_file], output="screen"
    )

    # 4. Robot State Publisher - generates URDF from xacro when started
    # This ensures fresh URDF is used each time
    robot_description_content = Command(
        ["xacro ", xacro_file, " use_gazebo:=true robot_name:=rosmaster_x3"]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                )
            },
        ],
    )

    # 5. Spawn Robot - spawns after RSP has published the URDF
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
            f'sdf_filename: "{urdf_file}", name: "rosmaster_x3", pose:{{position:{{x: 0.0, y: 0.0, z: 0.5}}}}',
        ],
        output="screen",
    )

    # 6. Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # 7. Controller Spawners
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

    # Event Handlers - proper sequencing:
    # URDF generation -> Fix paths -> Start RSP -> Spawn robot -> Controllers

    sed_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_urdf, on_exit=[replace_package_uri]
        )
    )

    # Start RSP after URDF is fixed, then spawn robot after delay
    rsp_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=replace_package_uri,
            on_exit=[
                robot_state_publisher,
                TimerAction(period=2.0, actions=[spawn_robot]),
            ],
        )
    )

    # Give gz_ros2_control 3 seconds to initialize after spawn, then start controllers
    controller_delay_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[spawn_joint_state_broadcaster],
                )
            ],
        )
    )

    drive_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_mecanumbot_drive_controller],
        )
    )

    return LaunchDescription(
        [
            set_ign_resource_path,
            set_ign_plugin_path,
            use_sim_time,
            generate_urdf,
            sed_event,
            rsp_event,
            ign_gazebo,
            bridge,
            controller_delay_event,
            drive_controller_event,
        ]
    )
