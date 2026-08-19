/**
 * @file driver_runtime.hpp
 * @brief ROS runtime entry point for a validated driver configuration.
 */

#ifndef DRIVER_RUNTIME_HPP
#define DRIVER_RUNTIME_HPP

#include "driver_config.hpp"

namespace dephan_ros {
/** Initialize ROS, apply ROS parameter overrides and run until shutdown. */
int run_driver_runtime(int argc, char* argv[], DriverConfig configuration);
} // namespace dephan_ros

#endif
