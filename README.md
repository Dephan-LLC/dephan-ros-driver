# ROS driver for DEPHAN LLC LiDars

## Supported ROS versions

This `ros2` branch contains the ROS 2 Jazzy driver.

| Branch | ROS distribution | Status |
| --- | --- | --- |
| `ros2` | [Jazzy](https://docs.ros.org/en/jazzy/index.html) | Current ROS 2 version |
| `ros1` | [Noetic](http://wiki.ros.org/noetic) | Maintained legacy version; Noetic is the final ROS 1 distribution |

## Documentation
For installation, building, running and API reference, see the [ROS Jazzy documentation](https://dephan-llc.github.io/dephan-ros-driver/jazzy/index.html). ROS 1 users should use the [`ros1` branch](https://github.com/Dephan-LLC/dephan-ros-driver/tree/ros1) and [ROS Noetic documentation](https://dephan-llc.github.io/dephan-ros-driver/noetic/index.html).

## Quick start

```bash
ros2 launch mech_lidar_driver udp.launch.py source_ip:=192.168.0.120 udp_port:=50007
```

The driver publishes `sensor_msgs/msg/LaserScan`,
`sensor_msgs/msg/PointCloud2` and a standard
`diagnostic_msgs/msg/DiagnosticArray`. Use
`capture_type:=SINGLE` for packet-level publication or keep the default
`FULL` for one 360-degree scan. See the documentation for PCAP and
multi-LiDAR launch examples.
