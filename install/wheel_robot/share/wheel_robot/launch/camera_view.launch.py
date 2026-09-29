"""Open the robot camera viewer without starting or controlling the chassis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "camera_viewer.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Camera viewer ROS parameter file",
            ),
            Node(
                package="wheel_robot",
                executable="camera_viewer.py",
                name="camera_viewer",
                output="screen",
                emulate_tty=True,
                parameters=[params_file],
            ),
        ]
    )
