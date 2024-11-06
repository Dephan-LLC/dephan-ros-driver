.. ROS driver for the DEPHAN-LLC LiDars documentation master file, created by
   sphinx-quickstart on Wed Oct 23 11:59:23 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Dephan ROS driver documentation
===============================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

.. toctree::
   :hidden:
   
   Installation <installation>

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

1. ``packet_handler_mech.hpp`` contais ...; 
2. ``packet_raw.hpp`` contais ...;
3. ``reciever_socket.hpp`` contains ...; 
4. ``ros_driver.hpp`` contains ...; 

Also, you can test your driver installation (as described in the :doc:`installation guide <installation>`) by running the 
testing ROS node ``dephan_node.cpp``.


Quick links
-----------
* :doc:`Installation <installation>`

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`
