=====================
DEPHAN ROS node howto
=====================


Description
-----------

The best way to use driver as you want is specify json configuration file for the driver operation.

There are two types of configuration files: for *udp operation* and for *pcap-based operation*.

For both types there is a ``"capture type"`` parameter which can take only two values: ``"FULL"`` and ``"SINGLE"``.

When using the ``"FULL"`` type, 360-degree view packets will be captured, whereas in the ``"SINGLE"`` mode, packets will be captured singly.

UDP operation
^^^^^^^^^^^^^

It is used to work with a real lidar device connected to the ROS node via UDP.

You can find default configurations for UDP mode by ``src/mech_lidar_driver/configs/default_udp_config.json``:

.. code-block:: javascript

    {
        "mode": "UDP",
        "name": "test_UDP",
        "ip": "192.168.0.120",
        "port": 50007,
        "topic": "laserscan_data_udp",
        "capture_type": "FULL"
    }

You can change the last 5 fields according to your preferences.

The ``ip`` field is used as a source-IP filter for incoming UDP packets.
Use ``"0.0.0.0"`` or an empty string to accept packets from any source.
This is useful in Docker Desktop/NAT environments where the visible source
address may differ from the LiDAR address.

Configuration validation
^^^^^^^^^^^^^^^^^^^^^^^^

The configuration is validated before ROS polling starts. ``mode``,
``capture_type`` and ``topic`` are always required. UDP mode also requires
``ip`` and ``port``; PCAP mode requires ``pcap_path``. Ports must be in the
``1..65535`` range, timeout and polling periods must be positive, and unknown
JSON keys are rejected to expose spelling errors.

JSON values provide defaults for runtime options. ROS 2 parameters declared by
the node override those runtime values. Command-line HTTP connection options
apply only when an HTTP CLI command is executed.

The ROS 2 executor handles ROS callbacks while UDP or PCAP polling runs in a
dedicated worker thread. Polling exceptions shut down the ROS context and make
the process return a non-zero result.

In ``FULL`` capture mode the driver groups packets by the firmware ``ROT``
counter and publishes only after all 20 unique ``ENC`` positions are present.
An incomplete revolution is discarded when a newer counter arrives.

PCAP-based operation
^^^^^^^^^^^^^^^^^^^^

It is used when there is no real device at hand, but there is a file with recorded traffic from it.

You can find default configurations for UDP mode by ``src/mech_lidar_driver/configs/default_pcap_config.json``:

.. code-block:: javascript

    {
        "mode": "PCAP",
        "name": "test_PCAP",
        "pcap_path": "/root/test.pcap",
        "topic": "laserscan_data_pcap",
        "capture_type": "FULL"
    }

You can change the last 4 fields according to your preferences.


Usage
-----

Usage scenarios
^^^^^^^^^^^^^^^

1. Show help information:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --help


2. Run node with default configuration by ``src/mech_lidar_driver/configs/default_udp_config.json``:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver

3. Run node with user-specified configuration:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --config <RELATIVE_PATH_TO_JSON_CONFIG>

4. Run HTTP API command without starting ROS polling:

.. code-block:: shell

    ros2 run mech_lidar_driver mech_driver --lidar-ip 192.168.0.120 --get-config

See :doc:`LiDAR HTTP API CLI <http_api>` for the full command list.

Launch files
^^^^^^^^^^^^

The package provides three launch files:

* ``driver.launch.py`` exposes all parameters and remains the generic entry point;
* ``udp.launch.py`` selects the default UDP configuration;
* ``pcap.launch.py`` selects the default PCAP configuration.

Start a full UDP scan:

.. code-block:: shell

    ros2 launch mech_lidar_driver udp.launch.py \
      source_ip:=192.168.0.120 udp_port:=50007 \
      scan_topic:=scan pointcloud_topic:=points frame_id:=base_link

Publish each 115-point packet instead of a complete revolution:

.. code-block:: shell

    ros2 launch mech_lidar_driver udp.launch.py capture_type:=SINGLE

Replay a PCAP file:

.. code-block:: shell

    ros2 launch mech_lidar_driver pcap.launch.py \
      pcap_path:=/data/lidar.pcap capture_type:=FULL

For two lidars, each instance must use a unique node name, namespace, UDP port
and output topics:

.. code-block:: shell

    ros2 launch mech_lidar_driver udp.launch.py namespace:=front \
      node_name:=front_driver source_ip:=192.168.0.120 udp_port:=50007 \
      scan_topic:=scan pointcloud_topic:=points
    ros2 launch mech_lidar_driver udp.launch.py namespace:=rear \
      node_name:=rear_driver source_ip:=192.168.0.121 udp_port:=50008 \
      scan_topic:=scan pointcloud_topic:=points

Runtime parameters
^^^^^^^^^^^^^^^^^^

Declared ROS 2 parameters override values loaded from JSON. The generic launch
file exposes ``capture_type``, ``source_ip``,
``udp_port``, ``pcap_path``, ``scan_topic``,
``pointcloud_topic``, ``frame_id``, ``angle_offset_deg``,
``udp_reconnect_initial_ms``, ``udp_reconnect_max_ms`` and the HTTP/Safety
Zones settings. This ordering makes a validated JSON file
the base configuration while launch arguments configure a specific deployment.

Diagnostics
^^^^^^^^^^^

Diagnostics are enabled by default and published as
``diagnostic_msgs/msg/DiagnosticArray`` on ``diagnostics``. The status
contains the transport, age of the last scan, valid packet count, published
message count, invalid packet count, discarded incomplete revolutions and UDP
socket recovery state/counters. It
changes to ``WARN`` with ``No scan data`` after
``no_data_timeout_s`` (default 2 seconds). Use
``diagnostics_enabled``, ``diagnostics_topic``,
``diagnostics_period_s`` and ``no_data_timeout_s`` to configure
monitoring.

After a system error from ``poll`` or ``recvfrom``, the UDP socket is recreated
without stopping the node. Diagnostics changes to ``ERROR`` with
``UDP socket recovering`` while retries use exponential backoff from
``udp_reconnect_initial_ms`` (250 ms) up to ``udp_reconnect_max_ms`` (5 s).
Normal receive timeouts while the motor is stopped do not trigger recovery.

For UDP, ``hardware_id`` uses the configured source IP. When the source filter
is disabled with ``0.0.0.0`` for Docker/NAT, it uses ``http_host`` instead so
multiple devices remain distinguishable.


Operation testing
^^^^^^^^^^^^^^^^^

To ensure that driver works correctly you can capture publishing data.

1. For default configuration:

.. code-block:: shell

    ros2 topic echo --once /scan
    ros2 topic echo --once /points
    ros2 topic echo --once /diagnostics


2. For user-specified configuration:

.. code-block:: shell

    ros2 topic echo <JSON_CONFIG["topic"]>
