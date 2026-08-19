import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("mech_lidar_driver")
    default_config = os.path.join(
        share, "configs", "default_udp_config.json"
    )
    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("node_name", default_value="lidar_driver"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("capture_type", default_value="FULL"),
        DeclareLaunchArgument("source_ip", default_value="192.168.0.120"),
        DeclareLaunchArgument("udp_port", default_value="50007"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("pointcloud_topic", default_value="points"),
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
    forwarded = {
        "config": LaunchConfiguration("config"),
        "node_name": LaunchConfiguration("node_name"),
        "namespace": LaunchConfiguration("namespace"),
        "capture_type": LaunchConfiguration("capture_type"),
        "source_ip": LaunchConfiguration("source_ip"),
        "udp_port": LaunchConfiguration("udp_port"),
        "scan_topic": LaunchConfiguration("scan_topic"),
        "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
        "frame_id": LaunchConfiguration("frame_id"),
        "angle_offset_deg": LaunchConfiguration("angle_offset_deg"),
        "safety_debug": LaunchConfiguration("safety_debug"),
        "http_host": LaunchConfiguration("http_host"),
        "http_port": LaunchConfiguration("http_port"),
        "http_timeout_ms": LaunchConfiguration("http_timeout_ms"),
        "safety_zones_topic": LaunchConfiguration("safety_zones_topic"),
        "safety_zones_status_topic": LaunchConfiguration(
            "safety_zones_status_topic"
        ),
        "safety_debug_period_s": LaunchConfiguration("safety_debug_period_s"),
        "diagnostics_enabled": LaunchConfiguration("diagnostics_enabled"),
        "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
        "diagnostics_period_s": LaunchConfiguration("diagnostics_period_s"),
        "no_data_timeout_s": LaunchConfiguration("no_data_timeout_s"),
        "udp_reconnect_initial_ms": LaunchConfiguration(
            "udp_reconnect_initial_ms"
        ),
        "udp_reconnect_max_ms": LaunchConfiguration("udp_reconnect_max_ms"),
    }
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "driver.launch.py")),
        launch_arguments=forwarded.items(),
    )
    return LaunchDescription(arguments + [include])
