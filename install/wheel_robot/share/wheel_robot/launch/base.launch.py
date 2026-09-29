"""Start the ESP32 serial bridge for the wheel-legged chassis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Type-C serial device connected to the ESP32",
            ),
            Node(
                package="wheeled_legged_pkg",
                executable="wl_base_node",
                name="wl_base_node",
                output="screen",
                emulate_tty=True,
                parameters=[{"serial_port": serial_port}],
            ),
        ]
    )
