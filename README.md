# ROS driver for DEPHAN LLC LiDars

## Supported ROS versions

This `ros1` branch contains the ROS 1 Noetic driver. Noetic is the final ROS 1
distribution, so this branch is maintained as the legacy ROS 1 version.

| Branch | ROS distribution | Status |
| --- | --- | --- |
| `ros1` | [Noetic](http://wiki.ros.org/noetic) | Maintained legacy version |
| `ros2` | [Jazzy](https://docs.ros.org/en/jazzy/index.html) | Current ROS 2 version |

## Documentation
For installation, building, running and API reference, see the [ROS Noetic documentation](https://dephan-llc.github.io/dephan-ros-driver/noetic/index.html). ROS 2 users should use the [`ros2` branch](https://github.com/Dephan-LLC/dephan-ros-driver/tree/ros2) and [ROS Jazzy documentation](https://dephan-llc.github.io/dephan-ros-driver/jazzy/index.html).

## Quick start

```bash
roslaunch mech_lidar_driver udp.launch source_ip:=192.168.0.120 udp_port:=50007
```

The driver publishes `sensor_msgs/LaserScan`,
`sensor_msgs/PointCloud2` and a standard
`diagnostic_msgs/DiagnosticArray`. Use `capture_type:=SINGLE` for
packet-level publication or keep the default `FULL` for one 360-degree
scan. See the documentation for PCAP and multi-LiDAR launch examples.
