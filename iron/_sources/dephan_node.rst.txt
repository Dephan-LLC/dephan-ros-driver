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
Create 3 separeted terminals.

1. In tetminal 1: 

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        roscore
    
    .. code-tab:: shell ROS:iron 

        echo No need this terminal


2. In terminal 2:

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        rosrun mech_lidar_driver mech_driver

    .. code-tab:: shell ROS:iron

        ros2 run mech_lidar_driver mech_driver <PATH_TO_JSON_CONFIG>


3. In terminal 3:

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        rostopic echo point_cloud2_data

    .. code-tab:: shell ROS:iron

        ros2 topic echo point_cloud2_data


Now you should see data stream in the terminal 3.