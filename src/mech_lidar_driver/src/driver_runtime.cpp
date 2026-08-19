/**
 * @file driver_runtime.cpp
 * @brief ROS 2 lifecycle for the LiDAR driver.
 */

#include "driver_runtime.hpp"

#include "ros_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <thread>
#include <utility>

namespace dephan_ros {
namespace {
constexpr double PI = 3.14159265358979323846;

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
    rclcpp::init(argc, argv);
    const DriverRuntimeOptions runtime_options =
        make_runtime_options(configuration);

    std::shared_ptr<Driver> driver;
    const bool is_full = configuration.capture_type == CaptureType::FULL;
    if (configuration.mode == DriverMode::PCAP) {
        driver = std::make_shared<Driver>(
            configuration.pcap_path, configuration.topic, is_full,
            runtime_options
        );
    }
    else {
        driver = std::make_shared<Driver>(
            configuration.ip, configuration.port, configuration.topic,
            is_full, runtime_options
        );
    }

    const std::string capture_type =
        driver->get_parameter("capture_type").as_string();
    const std::string scan_topic =
        driver->get_parameter("scan_topic").as_string();
    const std::string frame_id = driver->get_parameter("frame_id").as_string();
    RCLCPP_INFO(
        driver->get_logger(),
        "Starting %s driver in %s mode; scan_topic=%s, frame_id=%s",
        to_string(configuration.mode), capture_type.c_str(),
        scan_topic.c_str(), frame_id.c_str()
    );

    int polling_result = 0;
    std::thread polling_thread([&]() {
        try {
            while (rclcpp::ok()) {
                if (capture_type == "FULL") {
                    driver->poll_full();
                }
                else {
                    driver->poll();
                }
            }
        }
        catch (const std::exception& ex) {
            polling_result = 1;
            RCLCPP_ERROR(
                driver->get_logger(), "Data polling failed: %s", ex.what()
            );
            rclcpp::shutdown();
        }
        catch (...) {
            polling_result = 1;
            RCLCPP_ERROR(driver->get_logger(), "Data polling failed");
            rclcpp::shutdown();
        }
    });

    rclcpp::spin(driver);
    rclcpp::shutdown();
    if (polling_thread.joinable()) {
        polling_thread.join();
    }
    return polling_result;
}
} // namespace dephan_ros
