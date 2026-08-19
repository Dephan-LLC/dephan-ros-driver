/**
 * @file driver_config.hpp
 * @brief Typed and validated driver configuration.
 */

#ifndef DRIVER_CONFIG_HPP
#define DRIVER_CONFIG_HPP

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>

namespace dephan_ros {
/** Input transport selected by the configuration. */
enum class DriverMode { UDP, PCAP };

/** ROS publication granularity selected by the configuration. */
enum class CaptureType { FULL, SINGLE };

/** Validated configuration used to construct the ROS driver. */
struct DriverConfig {
    /** Required transport and publication mode. */
    DriverMode mode;
    CaptureType capture_type;

    /** Instance label and transport-specific settings. */
    std::string name = "lidar_driver";
    std::string ip;
    uint16_t port = 0;
    std::string pcap_path;

    /** ROS output topics and coordinate frame settings. */
    std::string topic;
    std::string pointcloud_topic;
    std::string frame_id = "base_link";
    double angle_offset_deg = 90.0;

    /** Optional Safety Zones debug publication settings. */
    bool safety_debug = false;
    std::string safety_zones_topic = "safety_zones_markers";
    std::string safety_zones_status_topic = "safety_zones_status";
    double safety_debug_period_s = 1.0;

    /** Standard ROS diagnostics publication settings. */
    bool diagnostics_enabled = true;
    std::string diagnostics_topic = "diagnostics";
    double diagnostics_period_s = 1.0;
    double no_data_timeout_s = 2.0;

    /** UDP socket recreation backoff after poll/recv system errors. */
    int udp_reconnect_initial_ms = 250;
    int udp_reconnect_max_ms = 5000;

    /** HTTP connection used by Safety Zones debug polling. */
    std::string http_host = "192.168.0.120";
    uint16_t http_port = 80;
    int http_timeout_ms = 3000;
};

/** Parse a JSON object and reject missing, mistyped or unknown fields. */
DriverConfig parse_driver_config(const nlohmann::json& value);

/** Serialize a validated configuration for logging and parameter overlays. */
nlohmann::json driver_config_to_json(const DriverConfig& value);

const char* to_string(DriverMode value);
const char* to_string(CaptureType value);
} // namespace dephan_ros

#endif
