Driver runtime
==============

- Defined in ``mech_lidar_driver/include/driver_runtime.hpp``

The runtime keeps the ROS executor responsive and performs blocking UDP/PCAP
polling in a dedicated worker thread.

Functions
---------

.. doxygenfunction:: dephan_ros::run_driver_runtime
