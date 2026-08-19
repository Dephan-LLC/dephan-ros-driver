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

    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("node_name", default_value="lidar_driver"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("capture_type", default_value="FULL"),
        DeclareLaunchArgument("source_ip", default_value="192.168.0.120"),
        DeclareLaunchArgument("udp_port", default_value="50007"),
        DeclareLaunchArgument("pcap_path", default_value="/root/test.pcap"),
        DeclareLaunchArgument("scan_topic", default_value="laserscan_data_udp"),
        DeclareLaunchArgument(
            "pointcloud_topic", default_value="pointcloud_data_udp"
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
            "safety_zones_status_topic", default_value="safety_zones_status"
        ),
        DeclareLaunchArgument("safety_debug_period_s", default_value="1.0"),
        DeclareLaunchArgument("diagnostics_enabled", default_value="true"),
        DeclareLaunchArgument("diagnostics_topic", default_value="diagnostics"),
        DeclareLaunchArgument("diagnostics_period_s", default_value="1.0"),
        DeclareLaunchArgument("no_data_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("udp_reconnect_initial_ms", default_value="250"),
        DeclareLaunchArgument("udp_reconnect_max_ms", default_value="5000"),
    ]

    node = Node(
        package="mech_lidar_driver",
        executable="mech_driver",
        name=LaunchConfiguration("node_name"),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        arguments=["--config", LaunchConfiguration("config")],
        parameters=[
            {
                "capture_type": LaunchConfiguration("capture_type"),
                "source_ip": LaunchConfiguration("source_ip"),
                "udp_port": ParameterValue(
                    LaunchConfiguration("udp_port"), value_type=int
                ),
                "pcap_path": LaunchConfiguration("pcap_path"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "angle_offset_deg": ParameterValue(
                    LaunchConfiguration("angle_offset_deg"), value_type=float
                ),
                "safety_debug": ParameterValue(
                    LaunchConfiguration("safety_debug"), value_type=bool
                ),
                "http_host": LaunchConfiguration("http_host"),
                "http_port": ParameterValue(
                    LaunchConfiguration("http_port"), value_type=int
                ),
                "http_timeout_ms": ParameterValue(
                    LaunchConfiguration("http_timeout_ms"), value_type=int
                ),
                "safety_zones_topic": LaunchConfiguration(
                    "safety_zones_topic"
                ),
                "safety_zones_status_topic": LaunchConfiguration(
                    "safety_zones_status_topic"
                ),
                "safety_debug_period_s": ParameterValue(
                    LaunchConfiguration("safety_debug_period_s"),
                    value_type=float,
                ),
                "diagnostics_enabled": ParameterValue(
                    LaunchConfiguration("diagnostics_enabled"), value_type=bool
                ),
                "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                "diagnostics_period_s": ParameterValue(
                    LaunchConfiguration("diagnostics_period_s"), value_type=float
                ),
                "no_data_timeout_s": ParameterValue(
                    LaunchConfiguration("no_data_timeout_s"), value_type=float
                ),
                "udp_reconnect_initial_ms": ParameterValue(
                    LaunchConfiguration("udp_reconnect_initial_ms"),
                    value_type=int,
                ),
                "udp_reconnect_max_ms": ParameterValue(
                    LaunchConfiguration("udp_reconnect_max_ms"),
                    value_type=int,
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
