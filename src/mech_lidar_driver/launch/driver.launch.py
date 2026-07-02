import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("mech_lidar_driver"),
        "configs",
        "default_udp_config.json",
    )

    config = LaunchConfiguration("config")
    frame_id = LaunchConfiguration("frame_id")
    angle_offset_deg = LaunchConfiguration("angle_offset_deg")
    safety_debug = LaunchConfiguration("safety_debug")
    http_host = LaunchConfiguration("http_host")
    http_port = LaunchConfiguration("http_port")
    http_timeout_ms = LaunchConfiguration("http_timeout_ms")
    safety_zones_topic = LaunchConfiguration("safety_zones_topic")
    safety_zones_status_topic = LaunchConfiguration("safety_zones_status_topic")
    safety_debug_period_s = LaunchConfiguration("safety_debug_period_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to UDP/PCAP JSON config.",
            ),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("angle_offset_deg", default_value="90.0"),
            DeclareLaunchArgument("safety_debug", default_value="false"),
            DeclareLaunchArgument("http_host", default_value="192.168.0.120"),
            DeclareLaunchArgument("http_port", default_value="80"),
            DeclareLaunchArgument("http_timeout_ms", default_value="3000"),
            DeclareLaunchArgument(
                "safety_zones_topic", default_value="safety_zones_markers"
            ),
            DeclareLaunchArgument(
                "safety_zones_status_topic",
                default_value="safety_zones_status",
            ),
            DeclareLaunchArgument("safety_debug_period_s", default_value="1.0"),
            Node(
                package="mech_lidar_driver",
                executable="mech_driver",
                name="lidar_driver",
                output="screen",
                arguments=["--config", config],
                parameters=[
                    {
                        "frame_id": frame_id,
                        "angle_offset_deg": ParameterValue(
                            angle_offset_deg, value_type=float
                        ),
                        "safety_debug": ParameterValue(
                            safety_debug, value_type=bool
                        ),
                        "http_host": http_host,
                        "http_port": ParameterValue(http_port, value_type=int),
                        "http_timeout_ms": ParameterValue(
                            http_timeout_ms, value_type=int
                        ),
                        "safety_zones_topic": safety_zones_topic,
                        "safety_zones_status_topic": safety_zones_status_topic,
                        "safety_debug_period_s": ParameterValue(
                            safety_debug_period_s, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
