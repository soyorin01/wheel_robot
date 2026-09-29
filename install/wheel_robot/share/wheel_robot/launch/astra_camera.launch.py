"""启动 Astra Pro，并把默认测距窗口固定显示在车载 HDMI 屏幕。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
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
    default_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "astra_camera.yaml"]
    )
    workspace_library_path = PathJoinSubstitution(
        [FindPackagePrefix("wheel_robot"), "lib"]
    )
    show_display = LaunchConfiguration("show_display")
    start_depth_distance = LaunchConfiguration("start_depth_distance")
    local_display = LaunchConfiguration("local_display")
    local_xauthority = LaunchConfiguration("local_xauthority")

    camera_node = Node(
        package="wheel_robot",
        executable="astra_camera_node",
        name="astra_camera",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
    )

    depth_distance = Node(
        package="wheel_robot",
        executable="depth_distance_node",
        name="depth_distance",
        namespace="camera",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file")],
        condition=IfCondition(start_depth_distance),
        additional_env={
            "LD_LIBRARY_PATH": [
                workspace_library_path,
                TextSubstitution(text=":"),
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
            ]
        },
    )

    # 预览窗口固定使用香橙派本地桌面，不继承 SSH/X11 转发的 DISPLAY。
    # 因此即使从 MobaXterm 启动 launch，自动窗口仍出现在车载 HDMI 屏幕。
    depth_preview = Node(
        package="wheel_robot",
        executable="depth_preview.py",
        name="depth_preview",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(show_display),
        parameters=[
            {
                "show_display": ParameterValue(show_display, value_type=bool),
                "fullscreen": LaunchConfiguration("fullscreen"),
                "topic": LaunchConfiguration("viewer_topic"),
            }
        ],
        additional_env={
            "DISPLAY": local_display,
            "XAUTHORITY": local_xauthority,
            "QT_X11_NO_MITSHM": "1",
        },
    )

    shutdown_when_camera_stops = RegisterEventHandler(
        OnProcessExit(
            target_action=camera_node,
            on_exit=[Shutdown(reason="Astra camera node stopped")],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_display",
                default_value="true",
                description=(
                    "是否在车载 HDMI 屏幕弹出测距窗口；false 只关闭窗口，"
                    "不停止相机和 Topic"
                ),
            ),
            DeclareLaunchArgument(
                "start_depth_distance",
                default_value="true",
                description="是否启动深度障碍测距节点",
            ),
            DeclareLaunchArgument(
                "local_display",
                default_value=":0",
                description="车载 HDMI 所在的本地图形显示器",
            ),
            DeclareLaunchArgument(
                "fullscreen",
                default_value="true",
                description="车载测距窗口是否全屏显示",
            ),
            DeclareLaunchArgument(
                "local_xauthority",
                default_value="/home/orangepi/.Xauthority",
                description="香橙派本地桌面的 X11 授权文件",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Astra Pro ROS parameter file",
            ),
            DeclareLaunchArgument(
                "viewer_topic",
                default_value="/camera/depth/obstacle_view",
                description="测距可视化图像话题",
            ),
            LogInfo(
                msg=[
                    "Astra 测距 show_display=",
                    show_display,
                    "。true 时窗口固定显示在车载 HDMI ",
                    local_display,
                    "，MobaXterm 可另开 rqt_image_view 或 RViz2 查看 Topic。",
                ]
            ),
            camera_node,
            depth_distance,
            depth_preview,
            shutdown_when_camera_stops,
        ]
    )
