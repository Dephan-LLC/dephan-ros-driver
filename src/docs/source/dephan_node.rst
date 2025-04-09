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
        "port": 51551,
        "topic": "laserscan_data_udp", 
        "capture_type": "FULL"
    }

You can change the last 5 fields according to your preferences. 

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

Preparation 
^^^^^^^^^^^

To start using driver node in ros1 environment it is **neccesary** to create separeted terminal and run: 

.. code-block:: shell 

    roscore

After that you can create another terminal and start using driver node.

Usage scenarios
^^^^^^^^^^^^^^^

1. Show help information:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --help

2. Run node with default configuration by ``src/mech_lidar_driver/configs/default_udp_config.json``:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver

3. Run node with user-specified configuration:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver --config <RELATIVE_PATH_TO_JSON_CONFIG>

Operation testing
^^^^^^^^^^^^^^^^^

To ensure that driver works correctly you can capture publishing data:

1. For default configuration: 

.. code-block:: shell

    rostopic echo laserscan_data_udp

2. For user-specified configuration:

.. code-block:: shell

    rostopic echo <JSON_CONFIG["topic"]>


If driver operates correctly you should see frequently updated data. 