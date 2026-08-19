cmake_minimum_required(VERSION 3.14.4)
project(mech_lidar_driver)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

## Compile as C++17 for structured bindings used by the node.
add_compile_options(-std=c++17)

find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

find_library(LIBTINS tins)

add_executable(mech_driver
  src/dephan_node.cpp
  src/driver_cli.cpp
  src/driver_runtime.cpp
  src/ros_driver.cpp
  src/packet_handler_mech.cpp
  src/full_scan_assembler.cpp
  src/driver_config.cpp
  src/driver_diagnostics.cpp
  src/safety_zone_markers.cpp
  src/reciever_socket.cpp
  src/http_client.cpp
  utils/colored_cout.cpp
)

ament_target_dependencies(mech_driver
  diagnostic_msgs
  rclcpp
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  visualization_msgs
)

target_include_directories(mech_driver PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/utils
  # /usr/include/pcl-1.12
)

target_link_libraries(mech_driver
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${LIBTINS}
  nlohmann_json::nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(packet_handler_mech_test
    test/packet_handler_mech_test.cpp
    src/packet_handler_mech.cpp
    src/full_scan_assembler.cpp
  )
  target_include_directories(packet_handler_mech_test PRIVATE include)

  ament_add_gtest(driver_config_test
    test/driver_config_test.cpp
    src/driver_config.cpp
  )
  target_include_directories(driver_config_test PRIVATE include)
  target_link_libraries(driver_config_test nlohmann_json::nlohmann_json)

  ament_add_gtest(driver_cli_test
    test/driver_cli_test.cpp
    src/driver_cli.cpp
    src/http_client.cpp
  )
  target_include_directories(driver_cli_test PRIVATE include)
  target_link_libraries(driver_cli_test nlohmann_json::nlohmann_json)

  ament_add_gtest(http_client_test
    test/http_client_test.cpp
    src/http_client.cpp
  )
  target_include_directories(http_client_test PRIVATE include)

  ament_add_gtest(receiver_socket_test
    test/receiver_socket_test.cpp
    src/reciever_socket.cpp
  )
  target_include_directories(receiver_socket_test PRIVATE include)

  ament_add_gtest(driver_diagnostics_test
    test/driver_diagnostics_test.cpp
    src/driver_diagnostics.cpp
  )
  target_include_directories(driver_diagnostics_test PRIVATE include)

  ament_add_gtest(safety_zone_markers_test
    test/safety_zone_markers_test.cpp
    src/safety_zone_markers.cpp
  )
  target_include_directories(safety_zone_markers_test PRIVATE include)
  ament_target_dependencies(safety_zone_markers_test
    geometry_msgs
    rclcpp
    visualization_msgs
  )
  target_link_libraries(safety_zone_markers_test nlohmann_json::nlohmann_json)
endif()

install(TARGETS
  mech_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  configs
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
