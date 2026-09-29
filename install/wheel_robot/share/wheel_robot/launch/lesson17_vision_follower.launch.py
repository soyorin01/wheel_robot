"""Start the complete ROS 2 Lesson 17 multi-target follower."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("wheel_robot")
    params_file = LaunchConfiguration("params_file")
    start_base = LaunchConfiguration("start_base")
    start_camera = LaunchConfiguration("start_camera")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "base.launch.py"])
        ),
        condition=IfCondition(start_base),
        launch_arguments={"serial_port": LaunchConfiguration("serial_port")}.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "astra_camera.launch.py"])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "params_file": params_file,
            "start_depth_distance": "false",
            "show_display": "false",
            "local_display": LaunchConfiguration("local_display"),
            "local_xauthority": LaunchConfiguration("local_xauthority"),
        }.items(),
    )
    follower = Node(
        package="wheel_robot",
        executable="lesson17_vision_follower.py",
        name="lesson17_vision_follower",
        output="screen",
        emulate_tty=True,
        parameters=[
            params_file,
            {
                "enable_motion": ParameterValue(
                    LaunchConfiguration("enable_motion"), value_type=bool
                ),
                "show_debug": ParameterValue(
                    LaunchConfiguration("show_debug"), value_type=bool
                ),
                "target_mode": LaunchConfiguration("target_mode"),
            },
        ],
        additional_env={
            "DISPLAY": LaunchConfiguration("local_display"),
            "XAUTHORITY": LaunchConfiguration("local_xauthority"),
            "QT_X11_NO_MITSHM": "1",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "lesson17_vision_follower.yaml"]
                ),
                description="Lesson 17 camera and follower parameters",
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="false",
                description="Explicitly enable autonomous /cmd_vel output",
            ),
            DeclareLaunchArgument(
                "target_mode",
                default_value="person",
                description="person, red_cone, blue_cone or auto",
            ),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("show_debug", default_value="true"),
            DeclareLaunchArgument(
                "local_display",
                default_value=EnvironmentVariable("DISPLAY", default_value=":0"),
            ),
            DeclareLaunchArgument(
                "local_xauthority",
                default_value=EnvironmentVariable(
                    "XAUTHORITY", default_value="/home/orangepi/.Xauthority"
                ),
            ),
            base,
            camera,
            follower,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=follower,
                    on_exit=[Shutdown(reason="Lesson 17 follower stopped")],
                )
            ),
        ]
    )
