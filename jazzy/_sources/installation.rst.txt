==============================
Dephan ROS driver installation
==============================


Supported platfrorms
--------------------

This driver is developed and tested under the ``ROS:jazzy`` distribution.


Requirements
------------

1. Install `ROS:jazzy <https://docs.ros.org/en/jazzy/Installation.html>`_ packages.

2. Install ``pcl`` and ``pcl-ros``: 

.. code-block:: shell

    sudo apt update

    sudo apt install libpcl-dev ros-jazzy-pcl-conversions ros-jazzy-pcl-ros


3. Install ``tins``:

.. code-block:: shell

    sudo apt install libtins-dev


4. Install ``json``:

.. code-block:: shell 

    sudo apt install nlohmann-json3-dev 


Docker alternative
------------------

The repository contains a Docker setup for repeatable ROS:jazzy builds:

.. code-block:: shell

    cd docker
    docker compose build ros2-jazzy
    docker compose run --rm ros2-jazzy

See :doc:`Docker <docker>` for details.


Building
--------

To run and build driver please run the following commands:

.. code-block:: shell 

    cd ~ && git clone -b ros2 https://github.com/Dephan-LLC/dephan-ros-driver.git

    cd dephan-ros-driver

    rm -rf /tmp/dephan_ros2_ws
    mkdir -p /tmp/dephan_ros2_ws/src
    ln -s "$PWD/src/mech_lidar_driver" /tmp/dephan_ros2_ws/src/mech_lidar_driver

    cd /tmp/dephan_ros2_ws

    colcon build --symlink-install

    source install/setup.bash


Now you are ready to run the testing node. Please follow the :doc:`DEPHAN ROS node howto <dephan_node>` 
to learn how to test and use the driver.
