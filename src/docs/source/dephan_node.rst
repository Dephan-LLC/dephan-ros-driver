=====================
DEPHAN ROS node howto
=====================


Description
-----------

For the ``ROS:iron`` you can specify json configuration file for the driver operation. 

You can find default configurations both for PCAP and UDP modes respectively by: 

* ``mech_lidar_driver/configs/default_pcap_config.json``
* ``mech_lidar_driver/configs/default_udp_config.json``


Usage
-----

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        sudo rosrun mech_lidar_driver mech_driver

    .. code-tab:: shell ROS:iron

        sudo ros2 run mech_lidar_driver mech_driver <PATH_TO_JSON_CONFIG>