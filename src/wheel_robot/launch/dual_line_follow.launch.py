"""Start dual_line_follower.py with dual_line_follower.yaml."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_optional_bool(value: str):
    normalized = value.strip().lower()
    if normalized == "":
        return None
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"enable_motion must be true or false, got: {value!r}")


def _make_dual_line_follower_node(context, *_args, **_kwargs):
    params_file = LaunchConfiguration("params_file")
    parameters = [params_file]
    enable_motion = _as_optional_bool(
        LaunchConfiguration("enable_motion").perform(context)
    )
    if enable_motion is not None:
        parameters.append({"enable_motion": enable_motion})

    return [
        Node(
            package="wheel_robot",
            executable="dual_line_follower.py",
            name="dual_white_line_follower",
            output="screen",
            emulate_tty=True,
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "dual_line_follower.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Dual-white-line follower parameter file",
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="",
                description="Optional YAML override: true or false",
            ),
            OpaqueFunction(function=_make_dual_line_follower_node),
        ]
    )
