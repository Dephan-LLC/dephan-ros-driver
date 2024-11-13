=====================
DEPHAN ROS node howto
=====================


Description
-----------

You can specify json configuration file for the driver operation. 

You can find default configurations both for PCAP and UDP modes by, respectivly: 

* ``src/mech_lidar_driver/configs/default_pcap_config.json``
* ``src/mech_lidar_driver/configs/default_udp_config.json``


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


Operation testing
^^^^^^^^^^^^^^^^^

To ensure that driver works correctly you can capture publishing data. 

1. For default configuration:

.. code-block:: shell

    ros2 topic echo point_cloud2_data_udp


2. For user-specified configuration:

.. code-block:: shell 

    ros2 topic echo <JSON_CONFIG["topic"]>
