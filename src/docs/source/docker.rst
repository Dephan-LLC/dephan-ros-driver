======
Docker
======

Docker files are provided for repeatable ROS:iron builds.

Build image
-----------

.. code-block:: shell

    cd docker
    docker compose build ros2-iron

Run shell
---------

.. code-block:: shell

    cd docker
    docker compose run --rm ros2-iron

Build driver inside container
-----------------------------

.. code-block:: shell

    source /opt/ros/iron/setup.bash
    rm -rf /tmp/dephan_ros2_ws
    mkdir -p /tmp/dephan_ros2_ws/src
    ln -s /workspace/src/mech_lidar_driver /tmp/dephan_ros2_ws/src/mech_lidar_driver
    cd /tmp/dephan_ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ros2 run mech_lidar_driver mech_driver --help

Do not run ``colcon build`` directly from ``/workspace`` when the checkout
contains a ROS1/catkin top-level ``src/CMakeLists.txt`` file. A clean temporary
workspace avoids colcon treating that file as a package entry.

UDP and Docker Desktop
----------------------

Docker Desktop may rewrite the visible source address of incoming UDP packets.
If the driver receives packets but publishes no data, set ``ip`` in the JSON
configuration to ``"0.0.0.0"`` to disable the source-IP filter:

.. code-block:: json

    {
        "mode": "UDP",
        "ip": "0.0.0.0",
        "port": 50007,
        "topic": "laserscan_data_udp",
        "capture_type": "FULL"
    }
