==============================
Dephan ROS driver installation
==============================


Supported platfrorms
--------------------

Driver was tested under the ``Ubuntu 20.04`` and ``ROS Noetic`` distribution.

.. warning::
    This driver was tested only for `ros:noetic dockerhub image <https://hub.docker.com/_/ros>`_. 

    It is not recommended to use this driver under the other ros distribution.

Requirements
------------

Only the installed ros noetic package is enough.


Build and run
-------------

For run and build driver please run the following commands:

.. tabs::
    
    .. code-tab:: console Ubuntu

        git clone https://github.com/Dephan-LLC/dephan-ros-driver.git

        cd dephan-ros-driver

        catkin_make 

        source devel/setup.bash

Now you are ready to run the testing node:

1. In tetminal 1: 

.. tabs::
    
    .. code-tab:: console Ubuntu

        roscore

2. In terminal 2:

.. tabs::
    
    .. code-tab:: console Ubuntu

        rosrun mech_lidar_driver mech_driver

3. In terminal 3:

.. tabs::
    
    .. code-tab:: console Ubuntu

        rostopic echo point_cloud2_data

Now you should see data stream in the terminal 3.