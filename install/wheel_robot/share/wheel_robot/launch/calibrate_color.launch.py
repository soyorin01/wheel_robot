"""Calibrate the Astra Pro UVC color camera with a checkerboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "astra_camera.yaml"]
    )
    gui_environment = {
        "DISPLAY": LaunchConfiguration("display"),
        "XAUTHORITY": LaunchConfiguration("xauthority"),
        "XDG_RUNTIME_DIR": "/run/user/1000",
        "DBUS_SESSION_BUS_ADDRESS": "unix:path=/run/user/1000/bus",
    }

    camera_node = Node(
        package="wheel_robot",
        executable="astra_camera_node",
        name="astra_camera",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"enable_color": True, "enable_depth": False},
        ],
    )

    calibrator = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        name="astra_color_calibrator",
        output="screen",
        arguments=[
            "--size",
            LaunchConfiguration("checkerboard_size"),
            "--square",
            LaunchConfiguration("square_size"),
            "--camera_name",
            "astra_pro_color",
            "--no-service-check",
        ],
        remappings=[
            ("image", "/camera/color/image_raw"),
            ("camera", "/camera/color"),
        ],
        additional_env=gui_environment,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "checkerboard_size",
                default_value="9x6",
                description="Checkerboard inner-corner count",
            ),
            DeclareLaunchArgument(
                "square_size",
                default_value="0.025",
                description="Checkerboard square edge length in metres",
            ),
            DeclareLaunchArgument("display", default_value=":0"),
            DeclareLaunchArgument(
                "xauthority", default_value="/home/orangepi/.Xauthority"
            ),
            camera_node,
            calibrator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=camera_node,
                    on_exit=[Shutdown(reason="Astra color camera stopped")],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=calibrator,
                    on_exit=[Shutdown(reason="Color calibration window closed")],
                )
            ),
        ]
    )
