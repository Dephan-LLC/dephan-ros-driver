.. ROS driver for the DEPHAN-LLC LiDars documentation master file, created by
   sphinx-quickstart on Wed Oct 23 11:59:23 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Dephan ROS:noetic driver documentation
======================================

.. toctree::
   :maxdepth: 3
   :caption: Contents:

.. toctree::
   :hidden:
   
   Installation <installation>
   Docker <docker>

.. toctree::
   :hidden:

   DEPHAN ROS node howto <dephan_node>
   LiDAR HTTP API CLI <http_api>

.. toctree::
   :hidden:
   :caption: API 
   :titlesonly:
   :glob:

   cpp_src/ros_driver
   cpp_src/reciever_socket
   cpp_src/http_client
   cpp_src/driver_cli
   cpp_src/driver_runtime
   cpp_src/driver_config
   cpp_src/driver_diagnostics
   cpp_src/full_scan_assembler
   cpp_src/safety_zone_markers
   cpp_src/packet_handler_mech
   cpp_src/packet_raw


Quick start
-----------
To install all neccesary packages and setup driver please follow 
the :doc:`installation guide <installation>`. 


Project structure
-----------------
The purpose of this project is to provide convenient and easy-to-use ROS support for DEPHAN-LLC LiDars. 

There are some structural parts of the project:

1. ``packet_raw.hpp`` incapsulates all information about the raw packet recieved from the LiDar (or from the PCAP file);
2. ``packet_handler_mech.hpp`` contais methods for handling raw packages recieved from the LiDar (or from the PCAP file); 
3. ``driver_config.hpp`` validates JSON configuration into typed runtime values;
4. ``full_scan_assembler.hpp`` collects packets with one revolution counter and rejects incomplete revolutions;
5. ``reciever_socket.hpp`` contains methods for connecting and polling the LiDar device;
6. ``ros_driver.hpp`` contains methods for ROS operation;
7. ``http_client.hpp`` contains a minimal HTTP client used by LiDAR web API CLI commands;
8. ``driver_cli.hpp`` parses one-shot HTTP and driver command-line options;
9. ``driver_runtime.hpp`` owns ROS initialization, parameters and the spin lifecycle;
10. ``driver_diagnostics.hpp`` selects stable hardware identifiers for ROS diagnostics;
11. ``safety_zone_markers.hpp`` converts Safety Zones API responses into RViz markers.

Also, you can test your driver installation (as described in the :doc:`installation guide <installation>`) by running the 
testing ROS node ``dephan_node.cpp``.


DEPHAN ROS node usage
---------------------
Fot testing and using driver you should setup and run ROS node wich will process LiDar data and translate it to the ROS topic. 
To get information how to setup and use DEPHAN ROS node please follow the :doc:`DEPHAN ROS node howto <dephan_node>`.

LiDAR HTTP API usage
--------------------
The same executable can also be used as a command-line client for the LiDAR HTTP API.
HTTP commands return after the request is completed and do not start UDP/PCAP polling.
See :doc:`LiDAR HTTP API CLI <http_api>` for configuration, status, firmware, log and Safety zones commands.


Quick links
-----------
* :doc:`Installation <installation>`
* :doc:`Docker <docker>`
* :doc:`DEPHAN ROS node <dephan_node>`
* :doc:`LiDAR HTTP API CLI <http_api>`
