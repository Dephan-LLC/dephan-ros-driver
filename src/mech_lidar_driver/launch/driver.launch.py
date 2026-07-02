import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("mech_lidar_driver"),
        "configs",
        "default_udp_config.json",
    )

    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to UDP/PCAP JSON config.",
            ),
            Node(
                package="mech_lidar_driver",
                executable="mech_driver",
                name="lidar_driver",
                output="screen",
                arguments=["--config", config],
            ),
        ]
    )
