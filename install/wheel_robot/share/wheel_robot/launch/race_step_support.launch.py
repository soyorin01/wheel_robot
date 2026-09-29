"""比赛用：Astra 只开深度 + 5cm 台阶检测器。

目的：
1. wheelleg_race_v1_18_step.py 继续像现在一样自己打开 /dev/video0 做 RGB 循迹；
2. Astra ROS 节点只打开 depth，避免同时抢 Astra Pro 的彩色 UVC；
3. 只启动 step_detector_node.py，不启动 step_approach_test.py，
   因为接近/跳跃动作已经合入比赛循迹程序，避免两个节点同时发布 /cmd_vel；
4. 不启动 step_preview.py，减少比赛时 CPU/GPU/HDMI 开销。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    camera_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "astra_camera.yaml"]
    )
    step_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "step_approach_test.yaml"]
    )
    workspace_library_path = PathJoinSubstitution(
        [FindPackagePrefix("wheel_robot"), "lib"]
    )

    camera_node = Node(
        package="wheel_robot",
        executable="astra_camera_node",
        name="astra_camera",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("camera_params_file"),
            {
                # 关键：比赛循迹程序自己占用 RGB /dev/video0。
                # Astra ROS 这里只提供 step_detector 需要的深度流。
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
        parameters=[
            LaunchConfiguration("step_params_file"),
            {
                # 比赛时不需要 detector 预览图，减少计算和 DDS 带宽。
                "publish_debug_image": ParameterValue(
                    LaunchConfiguration("publish_debug_image"),
                    value_type=bool,
                ),
                "enable_file_log": ParameterValue(
                    LaunchConfiguration("enable_file_log"),
                    value_type=bool,
                ),
            },
        ],
    )

    shutdown_when_camera_stops = RegisterEventHandler(
        OnProcessExit(
            target_action=camera_node,
            on_exit=[Shutdown(reason="Astra depth camera stopped")],
        )
    )

    return LaunchDescription(
        [
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
                "publish_debug_image",
                default_value="false",
                description="Publish /step_detector/debug_image",
            ),
            DeclareLaunchArgument(
                "enable_file_log",
                default_value="true",
                description="Write step detector CSV log",
            ),
            camera_node,
            detector_node,
            shutdown_when_camera_stops,
        ]
    )
