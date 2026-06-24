======
Docker
======

Docker files are provided for repeatable ROS:noetic builds.

Build image
-----------

.. code-block:: shell

    cd docker
    docker compose build ros1-noetic

Run shell
---------

.. code-block:: shell

    cd docker
    docker compose run --rm --service-ports ros1-noetic

``--service-ports`` is important when using ``docker compose run`` because it
publishes the UDP port declared in ``compose.yml``.

Build driver inside container
-----------------------------

.. code-block:: shell

    source /opt/ros/noetic/setup.bash
    rm -rf /tmp/dephan_ws
    mkdir -p /tmp/dephan_ws/src
    ln -s /workspace/src/mech_lidar_driver /tmp/dephan_ws/src/mech_lidar_driver
    cd /tmp/dephan_ws
    catkin_make
    source devel/setup.bash
    rosrun mech_lidar_driver mech_driver --help

On Windows checkouts this temporary workspace avoids problems with the
``src/CMakeLists.txt`` catkin top-level file being stored as plain text instead
of a Linux symlink.

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
