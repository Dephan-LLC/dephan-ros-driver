/**
 * @file driver_runtime.hpp
 * @brief ROS runtime entry point for a validated driver configuration.
 */

#ifndef DRIVER_RUNTIME_HPP
#define DRIVER_RUNTIME_HPP

#include "driver_config.hpp"

namespace dephan_ros {
/**
 * Initialize ROS, apply parameter overrides and run until shutdown.
 *
 * ROS callbacks stay on the executor while UDP/PCAP polling runs in a
 * dedicated worker thread. A polling exception stops the ROS context and is
 * returned as a non-zero process result.
 */
int run_driver_runtime(int argc, char* argv[], DriverConfig configuration);
} // namespace dephan_ros

#endif
