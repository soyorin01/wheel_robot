"""Start the ICN race program with YAML tuning parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _optional_bool(value: str):
    normalized = value.strip().lower()
    if normalized == "":
        return None
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"expected true/false or empty value, got: {value!r}")


def _make_icn_node(context, *_args, **_kwargs):
    parameters = [LaunchConfiguration("params_file")]

    enable_motion = _optional_bool(LaunchConfiguration("enable_motion").perform(context))
    show_debug = _optional_bool(LaunchConfiguration("show_debug").perform(context))
    if enable_motion is not None:
        parameters.append({"runtime.enable_motion": enable_motion})
    if show_debug is not None:
        parameters.append({"runtime.show_debug": show_debug})

    return [
        Node(
            package="wheel_robot",
            executable="icn.py",
            name="wheelleg_vision_race",
            output="screen",
            emulate_tty=True,
            parameters=parameters,
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("wheel_robot")
    default_params = PathJoinSubstitution([package_share, "config", "icn.yaml"])
    camera_params = PathJoinSubstitution(
        [package_share, "config", "astra_camera.yaml"]
    )
    step_params = PathJoinSubstitution(
        [package_share, "config", "step_approach_test.yaml"]
    )

    base_node = Node(
        package="wheeled_legged_pkg",
        executable="wl_base_node",
        name="wl_base_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_base")),
        parameters=[{"serial_port": LaunchConfiguration("serial_port")}],
    )

    camera_node = Node(
        package="wheel_robot",
        executable="astra_camera_node",
        name="astra_camera",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_step_support")),
        parameters=[
            LaunchConfiguration("camera_params_file"),
            {
                "enable_color": False,
                "enable_depth": True,
            },
        ],
    )

    detector_node = Node(
        package="wheel_robot",
        executable="step_detector_node.py",
        name="step_detector",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_step_support")),
        parameters=[
            LaunchConfiguration("step_params_file"),
            {
                "publish_debug_image": ParameterValue(
                    LaunchConfiguration("publish_step_debug_image"),
                    value_type=bool,
                ),
                "enable_file_log": ParameterValue(
                    LaunchConfiguration("enable_step_file_log"),
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ICN race YAML parameter file",
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="",
                description="Optional YAML override: true or false",
            ),
            DeclareLaunchArgument(
                "show_debug",
                default_value="",
                description="Optional YAML override: true or false",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="false",
                description="Also start wl_base_node",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Type-C serial device connected to the ESP32",
            ),
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="Astra camera parameter file",
            ),
            DeclareLaunchArgument(
                "step_params_file",
                default_value=step_params,
                description="5cm step detector parameter file",
            ),
            DeclareLaunchArgument(
                "start_step_support",
                default_value="true",
                description="Start Astra depth camera and step_detector_node.py",
            ),
            DeclareLaunchArgument(
                "publish_step_debug_image",
                default_value="false",
                description="Publish /step_detector/debug_image",
            ),
            DeclareLaunchArgument(
                "enable_step_file_log",
                default_value="true",
                description="Write step detector CSV log",
            ),
            base_node,
            camera_node,
            detector_node,
            OpaqueFunction(function=_make_icn_node),
        ]
    )
