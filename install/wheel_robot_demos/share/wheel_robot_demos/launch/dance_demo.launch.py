from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_sequence = PathJoinSubstitution(
        [FindPackageShare("wheel_robot_demos"), "config", "dance_sequence.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="true only prints the sequence; false publishes robot commands",
            ),
            DeclareLaunchArgument(
                "sequence_file",
                default_value=default_sequence,
                description="YAML dance sequence file",
            ),
            DeclareLaunchArgument(
                "speed_scale",
                default_value="1.0",
                description="Values above 1.0 make the demo faster",
            ),
            DeclareLaunchArgument(
                "repeat",
                default_value="1",
                description="Override repeat count; 0 means use YAML value",
            ),
            DeclareLaunchArgument(
                "with_jump",
                default_value="false",
                description="Allow jump keyframes if the YAML contains jump: true",
            ),
            Node(
                package="wheel_robot_demos",
                executable="dance_demo.py",
                name="wheel_robot_dance_demo",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "sequence_file": LaunchConfiguration("sequence_file"),
                        "speed_scale": ParameterValue(
                            LaunchConfiguration("speed_scale"), value_type=float
                        ),
                        "repeat": ParameterValue(
                            LaunchConfiguration("repeat"), value_type=int
                        ),
                        "with_jump": ParameterValue(
                            LaunchConfiguration("with_jump"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
