cmake_minimum_required(VERSION 3.14.4)
project(mech_lidar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

find_library(LIBTINS tins)

add_executable(mech_driver
  src/dephan_node.cpp
  src/ros_driver.cpp
  src/packet_handler_mech.cpp
  src/reciever_socket.cpp
)

ament_target_dependencies(mech_driver
  rclcpp
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
)

target_include_directories(mech_driver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/include/pcl-1.12
)

target_link_libraries(mech_driver
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${LIBTINS}
)

install(TARGETS
  mech_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
