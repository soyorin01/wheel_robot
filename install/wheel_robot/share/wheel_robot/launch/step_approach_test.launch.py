"""Find a 5 cm step, approach the takeoff distance, then jump."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("wheel_robot"), "config", "step_approach_test.yaml"]
    )
    show_display = LaunchConfiguration("show_display")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Step approach test parameter file",
            ),
            DeclareLaunchArgument(
                "show_display",
                default_value="true",
                description="Open HDMI preview window",
            ),
            DeclareLaunchArgument(
                "local_display",
                default_value=":0",
                description="Onboard HDMI X11 display",
            ),
            DeclareLaunchArgument(
                "local_xauthority",
                default_value="/home/orangepi/.Xauthority",
                description="Onboard desktop X11 authority file",
            ),
            DeclareLaunchArgument(
                "fullscreen",
                default_value="true",
                description="Use fullscreen HDMI preview",
            ),
            DeclareLaunchArgument(
                "stop_distance_m",
                default_value="0.20",
                description="Takeoff distance before the step front edge",
            ),
            DeclareLaunchArgument(
                "search_speed_m_s",
                default_value="0.06",
                description="Slow forward speed while searching for a step",
            ),
            DeclareLaunchArgument(
                "approach_speed_m_s",
                default_value="0.50",
                description="Constant forward speed after the step distance is locked",
            ),
            DeclareLaunchArgument(
                "trigger_distance_source",
                default_value="commanded",
                description="commanded uses speed*time after lock; odom uses projected /odom distance",
            ),
            DeclareLaunchArgument(
                "jump_enabled",
                default_value="true",
                description="true jumps at the takeoff point; false stops there",
            ),
            DeclareLaunchArgument(
                "forward_speed_m_s",
                default_value="0.50",
                description="Forward speed during the jump pulse",
            ),
            DeclareLaunchArgument(
                "jump_min_height_m",
                default_value="0.18",
                description="Crouch height before jumping",
            ),
            DeclareLaunchArgument(
                "jump_max_height_m",
                default_value="0.30",
                description="Extension height during low jump",
            ),
            DeclareLaunchArgument(
                "landing_forward_seconds",
                default_value="2.0",
                description="How long to keep moving after the jump pulse",
            ),
            DeclareLaunchArgument(
                "enable_file_log",
                default_value="true",
                description="Write detector and approach CSV logs",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="/home/orangepi/wheel_robot/log/step_approach",
                description="Directory for detector and approach CSV logs",
            ),
            Node(
                package="wheel_robot",
                executable="step_detector_node.py",
                name="step_detector",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "enable_file_log": ParameterValue(
                            LaunchConfiguration("enable_file_log"), value_type=bool
                        ),
                        "log_dir": LaunchConfiguration("log_dir"),
                    },
                ],
            ),
            Node(
                package="wheel_robot",
                executable="step_approach_test.py",
                name="step_approach_test",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "stop_distance_m": ParameterValue(
                            LaunchConfiguration("stop_distance_m"), value_type=float
                        ),
                        "search_speed_m_s": ParameterValue(
                            LaunchConfiguration("search_speed_m_s"),
                            value_type=float,
                        ),
                        "approach_speed_m_s": ParameterValue(
                            LaunchConfiguration("approach_speed_m_s"),
                            value_type=float,
                        ),
                        "trigger_distance_source": LaunchConfiguration(
                            "trigger_distance_source"
                        ),
                        "jump_enabled": ParameterValue(
                            LaunchConfiguration("jump_enabled"), value_type=bool
                        ),
                        "forward_speed_m_s": ParameterValue(
                            LaunchConfiguration("forward_speed_m_s"),
                            value_type=float,
                        ),
                        "jump_min_height_m": ParameterValue(
                            LaunchConfiguration("jump_min_height_m"),
                            value_type=float,
                        ),
                        "jump_max_height_m": ParameterValue(
                            LaunchConfiguration("jump_max_height_m"),
                            value_type=float,
                        ),
                        "landing_forward_seconds": ParameterValue(
                            LaunchConfiguration("landing_forward_seconds"),
                            value_type=float,
                        ),
                        "enable_file_log": ParameterValue(
                            LaunchConfiguration("enable_file_log"), value_type=bool
                        ),
                        "log_dir": LaunchConfiguration("log_dir"),
                    },
                ],
            ),
            Node(
                package="wheel_robot",
                executable="step_preview.py",
                name="step_preview",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(show_display),
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "show_display": ParameterValue(show_display, value_type=bool),
                        "fullscreen": ParameterValue(
                            LaunchConfiguration("fullscreen"), value_type=bool
                        ),
                    },
                ],
                additional_env={
                    "DISPLAY": LaunchConfiguration("local_display"),
                    "XAUTHORITY": LaunchConfiguration("local_xauthority"),
                    "QT_X11_NO_MITSHM": "1",
                    "LD_LIBRARY_PATH": EnvironmentVariable(
                        "LD_LIBRARY_PATH", default_value=""
                    ),
                },
            ),
        ]
    )
