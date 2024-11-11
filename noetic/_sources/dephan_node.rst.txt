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

Create 3 separeted terminals.

1. In tetminal 1: 

.. code-block:: shell 

    roscore


2. In terminal 2:

.. code-block:: shell

    rosrun mech_lidar_driver mech_driver <PATH_TO_JSON_CONFIG>


3. In terminal 3: 

.. code-block:: shell

    rostopic echo point_cloud2_data


Now you should see data stream in the terminal 3.