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

Create 2 separeted terminals.

1. In tetminal 1: 

.. code-block:: shell 

    ros2 run mech_lidar_driver mech_driver <PATH_TO_JSON_CONFIG>


2. In terminal 2:

.. code-block:: shell

    ros2 topic echo point_cloud2_data_<JSON_CONFIG["topic"]>


Now you should see data stream in the terminal 2.