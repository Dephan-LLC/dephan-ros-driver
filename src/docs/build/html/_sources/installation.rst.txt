==============================
Dephan ROS driver installation
==============================


Supported platfrorms
--------------------

This driver was developed and tested under the ``ROS:noetic`` distribution for ``Ubuntu``. 


Requirements
------------

1. Install `ROS:noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ packages.

2. Install ``pcl`` and ``pcl-ros``: 

.. code-block:: shell

    sudo apt update

    sudo apt install libpcl-dev ros-noetic-pcl-conversions ros-noetic-pcl-ros


3. Install ``tins``:

.. code-block:: shell

    sudo apt install libtins-dev


4. Install ``json``:

.. code-block:: shell 

    sudo apt install nlohmann-json3-dev 


Building
--------

To run and build driver please run the following commands:

.. code-block:: shell 

    cd ~ && git clone -b ros1 https://github.com/Dephan-LLC/dephan-ros-driver.git

    cd dephan-ros-driver 

    catkin_make 

    source devel/setup.bash


Now you are ready to run the testing node. Please follow the :doc:`DEPHAN ROS node howto <dephan_node>` 
to learn how to test and use the driver.
