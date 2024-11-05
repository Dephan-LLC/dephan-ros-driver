==============================
Dephan ROS driver installation
==============================


Supported platfrorms
--------------------

Driver was tested under the ``ROS:noetic`` and ``ROS:iron`` distributions.


Requirements
------------

1. Install selected ros package (`ROS:noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ for ROS1 operation or `ROS:iron <https://docs.ros.org/en/iron/Installation.html>`_ for ROS2 operation).

2. Install ``PCL`` and ``PCL ROS``: 

.. code-block:: shell

    sudo apt install libpcl-dev

.. tabs::

    .. code-tab:: shell ROS:noetic

        sudo apt install ros-noetic-pcl-conversions ros-noetic-pcl-ros


    .. code-tab:: shell ROS:iron

        sudo apt install ros-iron-pcl-conversions ros-iron-pcl-ros


3. Install ``TINS``:

.. code-block:: shell

    sudo apt install libpcap-dev libssl-dev cmake

    cd ~ && git clone https://github.com/mfontanini/libtins.git

    cd libtins 

    mkdir build && cd build 
    
    cmake ../ 

    make && sudo make install


4. Install ``json``:

.. code-block:: shell 

    cd ~ && git clone https://github.com/nlohmann/json.git

    cd json 

    mkdir build && cd build 

    cmake ../ 

    make && sudo make install


Build and run
-------------

For run and build driver please run the following commands:

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        cd ~ && git clone -b main https://github.com/Dephan-LLC/dephan-ros-driver.git

        cd dephan-ros-driver

        catkin_make 

        source devel/setup.bash

    .. code-tab:: shell ROS:iron

        cd ~ && git clone -b ros2-hot https://github.com/Dephan-LLC/dephan-ros-driver.git

        cd dephan-ros-driver 

        colcon build 

        source install/setup.bash


Now you are ready to run the testing node. Create 3 terminals:

1. In tetminal 1: 

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        sudo roscore
    
    .. code-tab:: shell ROS:iron 

        echo No need this terminal


2. In terminal 2:

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        sudo rosrun mech_lidar_driver mech_driver

    .. code-tab:: shell ROS:iron

        sudo ros2 run mech_lidar_driver mech_driver


3. In terminal 3:

.. tabs::
    
    .. code-tab:: shell ROS:noetic

        sudo rostopic echo point_cloud2_data

    .. code-tab:: shell ROS:iron

        sudo ros2 topic echo point_cloud2_data


Now you should see data stream in the terminal 3.