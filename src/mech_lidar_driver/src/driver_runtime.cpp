/**
 * @file driver_runtime.cpp
 * @brief ROS 1 lifecycle and parameter handling for the LiDAR driver.
 */

#include "driver_runtime.hpp"

#include "ros_driver.hpp"

#include <nlohmann/json.hpp>
#include <ros/ros.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace dephan_ros {
namespace {
constexpr double PI = 3.14159265358979323846;

void apply_parameter_overrides(
    DriverConfig& configuration, ros::NodeHandle& nh,
    ros::NodeHandle& private_nh
) {
    nlohmann::json values = driver_config_to_json(configuration);

    auto string_parameter = [&](const char* parameter, const char* key) {
        std::string value = values.value(key, std::string());
        private_nh.param(parameter, value, value);
        values[key] = value;
    };
    auto bool_parameter = [&](const char* parameter, const char* key) {
        bool value = values.value(key, false);
        private_nh.param(parameter, value, value);
        values[key] = value;
    };
    auto int_parameter = [&](const char* parameter, const char* key) {
        int value = values.value(key, 0);
        private_nh.param(parameter, value, value);
        values[key] = value;
    };
    auto double_parameter = [&](const char* parameter, const char* key) {
        double value = values.value(key, 0.0);
        private_nh.param(parameter, value, value);
        values[key] = value;
    };

    string_parameter("mode", "mode");
    string_parameter("capture_type", "capture_type");
    string_parameter("source_ip", "ip");
    int_parameter("udp_port", "port");
    string_parameter("pcap_path", "pcap_path");
    string_parameter("scan_topic", "topic");
    string_parameter("pointcloud_topic", "pointcloud_topic");

    std::string frame_id = values.at("frame_id").get<std::string>();
    nh.param("frame_id", frame_id, frame_id);
    private_nh.param("frame_id", frame_id, frame_id);
    values["frame_id"] = frame_id;

    double angle_offset_deg = values.at("angle_offset_deg").get<double>();
    nh.param("angle_offset_deg", angle_offset_deg, angle_offset_deg);
    private_nh.param("angle_offset_deg", angle_offset_deg, angle_offset_deg);
    values["angle_offset_deg"] = angle_offset_deg;

    bool_parameter("safety_debug", "safety_debug");
    string_parameter("safety_zones_topic", "safety_zones_topic");
    string_parameter(
        "safety_zones_status_topic", "safety_zones_status_topic"
    );
    double_parameter("safety_debug_period_s", "safety_debug_period_s");
    string_parameter("http_host", "http_host");
    int_parameter("http_port", "http_port");
    int_parameter("http_timeout_ms", "http_timeout_ms");
    bool_parameter("diagnostics_enabled", "diagnostics_enabled");
    string_parameter("diagnostics_topic", "diagnostics_topic");
    double_parameter("diagnostics_period_s", "diagnostics_period_s");
    double_parameter("no_data_timeout_s", "no_data_timeout_s");
    int_parameter("udp_reconnect_initial_ms", "udp_reconnect_initial_ms");
    int_parameter("udp_reconnect_max_ms", "udp_reconnect_max_ms");

    configuration = parse_driver_config(values);
}

DriverRuntimeOptions make_runtime_options(const DriverConfig& configuration) {
    DriverRuntimeOptions options;
    options.pointcloud_topic = configuration.pointcloud_topic;
    options.frame_id = configuration.frame_id;
    options.angle_offset_rad = configuration.angle_offset_deg * PI / 180.0;
    options.safety_debug = configuration.safety_debug;
    options.http_host = configuration.http_host;
    options.http_port = configuration.http_port;
    options.http_timeout_ms = configuration.http_timeout_ms;
    options.safety_zones_topic = configuration.safety_zones_topic;
    options.safety_status_topic = configuration.safety_zones_status_topic;
    options.safety_debug_period_s = configuration.safety_debug_period_s;
    options.diagnostics_enabled = configuration.diagnostics_enabled;
    options.diagnostics_topic = configuration.diagnostics_topic;
    options.diagnostics_period_s = configuration.diagnostics_period_s;
    options.no_data_timeout_s = configuration.no_data_timeout_s;
    options.udp_reconnect_initial_ms = configuration.udp_reconnect_initial_ms;
    options.udp_reconnect_max_ms = configuration.udp_reconnect_max_ms;
    options.transport = to_string(configuration.mode);
    return options;
}
} // namespace

int run_driver_runtime(int argc, char* argv[], DriverConfig configuration) {
    ros::init(argc, argv, "lidar_driver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    apply_parameter_overrides(configuration, nh, private_nh);
    const DriverRuntimeOptions runtime_options =
        make_runtime_options(configuration);

    ROS_INFO_STREAM(
        "Starting " << to_string(configuration.mode) << " driver in "
        << to_string(configuration.capture_type) << " mode; scan_topic="
        << configuration.topic << ", frame_id=" << configuration.frame_id
    );

    std::unique_ptr<Driver> driver;
    if (configuration.mode == DriverMode::PCAP) {
        driver = std::make_unique<Driver>(
            nh, configuration.pcap_path, configuration.topic, runtime_options
        );
    }
    else {
        driver = std::make_unique<Driver>(
            nh, configuration.ip, configuration.port, configuration.topic,
            runtime_options
        );
    }

    while (ros::ok()) {
        if (configuration.capture_type == CaptureType::FULL) {
            driver->poll_full();
        }
        else {
            driver->poll();
        }
        ros::spinOnce();
    }
    return 0;
}
} // namespace dephan_ros
