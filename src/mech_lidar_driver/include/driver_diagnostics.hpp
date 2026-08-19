/**
 * @file driver_diagnostics.hpp
 * @brief Pure helpers for standard ROS diagnostics.
 */

#ifndef DRIVER_DIAGNOSTICS_HPP
#define DRIVER_DIAGNOSTICS_HPP

#include <string>

namespace dephan_ros {
/**
 * Select a stable hardware identifier for UDP or PCAP diagnostics.
 *
 * @param source_ip Configured UDP source filter.
 * @param pcap_path Configured PCAP path, empty in UDP mode.
 * @param http_host LiDAR HTTP address used when UDP source filtering is off.
 */
std::string diagnostic_hardware_id(
    const std::string& source_ip, const std::string& pcap_path,
    const std::string& http_host
);
} // namespace dephan_ros

#endif
