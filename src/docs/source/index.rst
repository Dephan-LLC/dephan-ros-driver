.. ROS driver for the DEPHAN-LLC LiDars documentation master file, created by
   sphinx-quickstart on Wed Oct 23 11:59:23 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Dephan ROS:iron driver documentation
===============================

.. toctree::
   :maxdepth: 3
   :caption: Contents:

.. toctree::
   :hidden:
   
   Installation <installation>

.. toctree::
   :hidden:

   DEPHAN ROS node howto <dephan_node>

.. toctree::
   :hidden:
   :caption: API 
   :titlesonly:
   :glob:

   cpp_src/ros_driver
   cpp_src/reciever_socket
   cpp_src/packet_handler_mech
   cpp_src/packet_raw


Quick start
-----------
To install all neccesary packages and setup driver please follow 
the :doc:`installation guide <installation>`. 


Project strucrute
-----------------
The purpose of this project is to provide convenient and easy-to-use ROS support for DEPHAN-LLC LiDars. 

There are some structural parts of the project:

1. ``packet_raw.hpp`` incapsulates all information about the raw packet recieved from the LiDar (or from the PCAP file);
2. ``packet_handler_mech.hpp`` contais methods for handling raw packages recieved from the LiDar (or from the PCAP file); 
3. ``reciever_socket.hpp`` contains methods for connecting and polling the LiDar device; 
4. ``ros_driver.hpp`` contains methods for ROS operation; 

Also, you can test your driver installation (as described in the :doc:`installation guide <installation>`) by running the 
testing ROS node ``dephan_node.cpp``.


DEPHAN ROS node usage
--------------
Fot testing and using driver you should setup and run ROS node wich will process LiDar data and translate it to the ROS topic. 
To get information how to setup and use DEPHAN ROS node please follow the :doc:`DEPHAN ROS node howto <dephan_node>`.


Quick links
-----------
* :doc:`Installation <installation>`
* :doc:`DEPHAN ROS node <dephan_node>`

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`
