/**
 * @file driver_config.cpp
 * @brief Typed and validated driver configuration.
 */

#include "driver_config.hpp"

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace dephan_ros {
namespace {
using json = nlohmann::json;

const std::set<std::string> ALLOWED_FIELDS = {
    "mode", "capture_type", "name", "ip", "port", "pcap_path", "topic",
    "pointcloud_topic", "frame_id", "angle_offset_deg", "safety_debug",
    "safety_zones_topic", "safety_zones_status_topic",
    "safety_debug_period_s", "diagnostics_enabled", "diagnostics_topic",
    "diagnostics_period_s", "no_data_timeout_s", "http_host", "http_port",
    "http_timeout_ms", "udp_reconnect_initial_ms", "udp_reconnect_max_ms"
};

template<typename T>
T required(const json& value, const char* key) {
    if (!value.contains(key)) {
        throw std::invalid_argument(
            std::string("Missing required configuration field: ") + key
        );
    }
    try {
        return value.at(key).get<T>();
    }
    catch (const json::exception&) {
        throw std::invalid_argument(
            std::string("Invalid type for configuration field: ") + key
        );
    }
}

template<typename T>
T optional(const json& value, const char* key, const T& default_value) {
    if (!value.contains(key)) {
        return default_value;
    }
    try {
        return value.at(key).get<T>();
    }
    catch (const json::exception&) {
        throw std::invalid_argument(
            std::string("Invalid type for configuration field: ") + key
        );
    }
}

void require_not_empty(const std::string& value, const char* key) {
    if (value.empty()) {
        throw std::invalid_argument(
            std::string("Configuration field must not be empty: ") + key
        );
    }
}

int integer_value(
    const json& value, const char* key, bool required_field, int default_value
) {
    if (!value.contains(key)) {
        if (required_field) {
            throw std::invalid_argument(
                std::string("Missing required configuration field: ") + key
            );
        }
        return default_value;
    }
    if (!value.at(key).is_number_integer() &&
        !value.at(key).is_number_unsigned()) {
        throw std::invalid_argument(
            std::string("Invalid type for configuration field: ") + key
        );
    }
    try {
        const int64_t result = value.at(key).get<int64_t>();
        if (result < std::numeric_limits<int>::min() ||
            result > std::numeric_limits<int>::max()) {
            throw std::invalid_argument(
                std::string("Integer configuration field is out of range: ") +
                key
            );
        }
        return static_cast<int>(result);
    }
    catch (const json::exception&) {
        throw std::invalid_argument(
            std::string("Integer configuration field is out of range: ") + key
        );
    }
}

uint16_t parse_port(
    const json& value, const char* key, bool required_field, int default_value
) {
    const int result = integer_value(value, key, required_field, default_value);
    if (result < 1 || result > 65535) {
        throw std::invalid_argument(
            std::string("Configuration field must be in range 1..65535: ") + key
        );
    }
    return static_cast<uint16_t>(result);
}
} // namespace

DriverConfig parse_driver_config(const nlohmann::json& value) {
    if (!value.is_object()) {
        throw std::invalid_argument("Driver configuration must be a JSON object");
    }
    for (const auto& item : value.items()) {
        if (ALLOWED_FIELDS.count(item.key()) == 0) {
            throw std::invalid_argument(
                "Unknown configuration field: " + item.key()
            );
        }
    }

    DriverConfig result;
    const std::string mode = required<std::string>(value, "mode");
    if (mode == "UDP") {
        result.mode = DriverMode::UDP;
    }
    else if (mode == "PCAP") {
        result.mode = DriverMode::PCAP;
    }
    else {
        throw std::invalid_argument("Configuration field mode must be UDP or PCAP");
    }

    const std::string capture_type =
        required<std::string>(value, "capture_type");
    if (capture_type == "FULL") {
        result.capture_type = CaptureType::FULL;
    }
    else if (capture_type == "SINGLE") {
        result.capture_type = CaptureType::SINGLE;
    }
    else {
        throw std::invalid_argument(
            "Configuration field capture_type must be FULL or SINGLE"
        );
    }

    result.name = optional<std::string>(value, "name", result.name);
    result.topic = required<std::string>(value, "topic");
    result.pointcloud_topic = optional<std::string>(
        value, "pointcloud_topic", result.pointcloud_topic
    );
    result.frame_id = optional<std::string>(value, "frame_id", result.frame_id);
    result.angle_offset_deg = optional<double>(
        value, "angle_offset_deg", result.angle_offset_deg
    );
    result.safety_debug = optional<bool>(
        value, "safety_debug", result.safety_debug
    );
    result.safety_zones_topic = optional<std::string>(
        value, "safety_zones_topic", result.safety_zones_topic
    );
    result.safety_zones_status_topic = optional<std::string>(
        value, "safety_zones_status_topic", result.safety_zones_status_topic
    );
    result.safety_debug_period_s = optional<double>(
        value, "safety_debug_period_s", result.safety_debug_period_s
    );
    result.diagnostics_enabled = optional<bool>(
        value, "diagnostics_enabled", result.diagnostics_enabled
    );
    result.diagnostics_topic = optional<std::string>(
        value, "diagnostics_topic", result.diagnostics_topic
    );
    result.diagnostics_period_s = optional<double>(
        value, "diagnostics_period_s", result.diagnostics_period_s
    );
    result.no_data_timeout_s = optional<double>(
        value, "no_data_timeout_s", result.no_data_timeout_s
    );
    result.udp_reconnect_initial_ms = integer_value(
        value, "udp_reconnect_initial_ms", false,
        result.udp_reconnect_initial_ms
    );
    result.udp_reconnect_max_ms = integer_value(
        value, "udp_reconnect_max_ms", false, result.udp_reconnect_max_ms
    );
    result.http_host = optional<std::string>(
        value, "http_host", result.http_host
    );
    result.http_port = parse_port(
        value, "http_port", false, result.http_port
    );
    result.http_timeout_ms = integer_value(
        value, "http_timeout_ms", false, result.http_timeout_ms
    );

    require_not_empty(result.name, "name");
    require_not_empty(result.topic, "topic");
    require_not_empty(result.frame_id, "frame_id");
    require_not_empty(result.safety_zones_topic, "safety_zones_topic");
    require_not_empty(
        result.safety_zones_status_topic, "safety_zones_status_topic"
    );
    require_not_empty(result.http_host, "http_host");
    require_not_empty(result.diagnostics_topic, "diagnostics_topic");
    if (!std::isfinite(result.angle_offset_deg)) {
        throw std::invalid_argument(
            "Configuration field must be finite: angle_offset_deg"
        );
    }
    if (!std::isfinite(result.safety_debug_period_s) ||
        result.safety_debug_period_s <= 0.0) {
        throw std::invalid_argument(
            "Configuration field must be positive: safety_debug_period_s"
        );
    }
    if (result.http_timeout_ms <= 0) {
        throw std::invalid_argument(
            "Configuration field must be positive: http_timeout_ms"
        );
    }
    if (!std::isfinite(result.diagnostics_period_s) ||
        result.diagnostics_period_s <= 0.0) {
        throw std::invalid_argument(
            "Configuration field must be positive: diagnostics_period_s"
        );
    }
    if (!std::isfinite(result.no_data_timeout_s) ||
        result.no_data_timeout_s <= 0.0) {
        throw std::invalid_argument(
            "Configuration field must be positive: no_data_timeout_s"
        );
    }
    if (result.udp_reconnect_initial_ms <= 0) {
        throw std::invalid_argument(
            "Configuration field must be positive: udp_reconnect_initial_ms"
        );
    }
    if (result.udp_reconnect_max_ms < result.udp_reconnect_initial_ms) {
        throw std::invalid_argument(
            "Configuration field udp_reconnect_max_ms must be greater than "
            "or equal to udp_reconnect_initial_ms"
        );
    }

    if (result.mode == DriverMode::UDP) {
        result.ip = required<std::string>(value, "ip");
        result.port = parse_port(value, "port", true, 0);
    }
    else {
        result.pcap_path = required<std::string>(value, "pcap_path");
        require_not_empty(result.pcap_path, "pcap_path");
    }

    return result;
}

nlohmann::json driver_config_to_json(const DriverConfig& value) {
    nlohmann::json result = {
        {"mode", to_string(value.mode)},
        {"capture_type", to_string(value.capture_type)},
        {"name", value.name},
        {"topic", value.topic},
        {"pointcloud_topic", value.pointcloud_topic},
        {"frame_id", value.frame_id},
        {"angle_offset_deg", value.angle_offset_deg},
        {"safety_debug", value.safety_debug},
        {"safety_zones_topic", value.safety_zones_topic},
        {"safety_zones_status_topic", value.safety_zones_status_topic},
        {"safety_debug_period_s", value.safety_debug_period_s},
        {"diagnostics_enabled", value.diagnostics_enabled},
        {"diagnostics_topic", value.diagnostics_topic},
        {"diagnostics_period_s", value.diagnostics_period_s},
        {"no_data_timeout_s", value.no_data_timeout_s},
        {"udp_reconnect_initial_ms", value.udp_reconnect_initial_ms},
        {"udp_reconnect_max_ms", value.udp_reconnect_max_ms},
        {"http_host", value.http_host},
        {"http_port", value.http_port},
        {"http_timeout_ms", value.http_timeout_ms},
    };
    if (value.mode == DriverMode::UDP) {
        result["ip"] = value.ip;
        result["port"] = value.port;
    }
    else {
        result["pcap_path"] = value.pcap_path;
    }
    return result;
}

const char* to_string(DriverMode value) {
    return value == DriverMode::UDP ? "UDP" : "PCAP";
}

const char* to_string(CaptureType value) {
    return value == CaptureType::FULL ? "FULL" : "SINGLE";
}
} // namespace dephan_ros
