"""Start the outdoor-track follower (chassis must already be running)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    enable_motion = LaunchConfiguration("enable_motion")
    default_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "shiwai.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Outdoor-track follower parameter file",
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="false",
                description="Explicitly enable /cmd_vel after dry-run testing",
            ),
            Node(
                package="wheel_robot",
                executable="shiwai.py",
                name="shiwai",
                output="screen",
                emulate_tty=True,
                parameters=[
                    params_file,
                    {
                        "enable_motion": ParameterValue(
                            enable_motion, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
