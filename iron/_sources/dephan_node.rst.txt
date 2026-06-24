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
        "port": 50001,
        "topic": "laserscan_data_udp",
        "capture_type": "FULL"
    }

You can change the last 5 fields according to your preferences.

The ``ip`` field is used as a source-IP filter for incoming UDP packets.
Use ``"0.0.0.0"`` or an empty string to accept packets from any source.
This is useful in Docker Desktop/NAT environments where the visible source
address may differ from the LiDAR address.

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


Operation testing
^^^^^^^^^^^^^^^^^

To ensure that driver works correctly you can capture publishing data.

1. For default configuration:

.. code-block:: shell

    ros2 topic echo point_cloud2_data_udp


2. For user-specified configuration:

.. code-block:: shell

    ros2 topic echo <JSON_CONFIG["topic"]>
