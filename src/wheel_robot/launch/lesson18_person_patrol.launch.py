"""Start the complete Lesson 18 person-aware autonomous patrol demo."""

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
            # Lesson 18 has its own RGB status window; suppress the depth-only window.
            "show_display": "false",
            "local_display": LaunchConfiguration("local_display"),
            "local_xauthority": LaunchConfiguration("local_xauthority"),
        }.items(),
    )
    patrol = Node(
        package="wheel_robot",
        executable="lesson18_person_patrol.py",
        name="lesson18_person_patrol",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
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
                    [package_share, "config", "lesson18_person_patrol.yaml"]
                ),
                description="Lesson 18 shared camera, obstacle and patrol parameters",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="true",
                description="Start the wheel-legged serial base bridge",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="Start Astra color/depth and obstacle nodes",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="ESP32 serial port",
            ),
            DeclareLaunchArgument(
                "local_display",
                default_value=EnvironmentVariable("DISPLAY", default_value=":0"),
                description="Display used for the RGB status window",
            ),
            DeclareLaunchArgument(
                "local_xauthority",
                default_value=EnvironmentVariable(
                    "XAUTHORITY", default_value="/home/orangepi/.Xauthority"
                ),
                description="X11 authorization file",
            ),
            base,
            camera,
            patrol,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=patrol,
                    on_exit=[Shutdown(reason="Lesson 18 patrol node stopped")],
                )
            ),
        ]
    )
