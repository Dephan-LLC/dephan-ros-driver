/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.cpp
 * @brief ROS driver for DEPHAN LLC LiDars
 */

#include "ros_driver.hpp"
#include "driver_diagnostics.hpp"
#include "full_scan_assembler.hpp"
#include "http_client.hpp"
#include "safety_zone_markers.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace dephan_ros {
namespace {
using json = nlohmann::json;

const float PI = 3.14159265358979323846f;
const int POINTS_PER_REV = pkt_hdl_Mech::POINTS_PER_REV;

std::string default_pointcloud_topic(const std::string& laserscan_topic) {
    std::string result = laserscan_topic;
    size_t pos = result.find("laserscan");
    if (pos != std::string::npos) {
        result.replace(pos, std::string("laserscan").size(), "pointcloud");
        return result;
    }
    return result + "_pointcloud";
}

float range_min_or_default(const std::vector<float>& ranges) {
    float result = std::numeric_limits<float>::infinity();
    for (float range : ranges) {
        if (std::isfinite(range) && range < result) {
            result = range;
        }
    }
    return std::isfinite(result) ? result : 0.0f;
}

float range_max_or_default(const std::vector<float>& ranges) {
    float result = 0.0f;
    for (float range : ranges) {
        if (std::isfinite(range) && range > result) {
            result = range;
        }
    }
    return result;
}

sensor_msgs::msg::PointCloud2 make_pointcloud_msg(
    const sensor_msgs::msg::LaserScan& scan
) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(scan.ranges.size());
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(scan.ranges.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < scan.ranges.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
        float range = scan.ranges[i];
        if (!std::isfinite(range)) {
            *iter_x = std::numeric_limits<float>::quiet_NaN();
            *iter_y = std::numeric_limits<float>::quiet_NaN();
            *iter_z = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        float angle = scan.angle_min +
                      static_cast<float>(i) * scan.angle_increment;
        *iter_x = range * std::cos(angle);
        *iter_y = range * std::sin(angle);
        *iter_z = 0.0f;
    }

    return cloud;
}

} // namespace

Driver::Driver(
    std::string ip_addr, unsigned port, std::string cloud_topic, bool is_full,
    DriverRuntimeOptions options
) : Node("driver"), ip_addr(ip_addr), port(port), is_full(is_full),
    runtime_options(std::move(options)),
    started_at(std::chrono::steady_clock::now()) {

    this->ip_addr = this->declare_parameter("source_ip", this->ip_addr);
    const int udp_port = this->declare_parameter(
        "udp_port", static_cast<int>(this->port)
    );
    if (udp_port < 1 || udp_port > 65535) {
        throw std::invalid_argument("udp_port must be in range 1..65535");
    }
    this->port = static_cast<unsigned>(udp_port);
    cloud_topic = this->declare_parameter("scan_topic", cloud_topic);
    const std::string capture_type = this->declare_parameter(
        "capture_type", this->is_full ? "FULL" : "SINGLE"
    );
    if (capture_type != "FULL" && capture_type != "SINGLE") {
        throw std::invalid_argument("capture_type must be FULL or SINGLE");
    }
    this->is_full = capture_type == "FULL";

    runtime_options.pointcloud_topic = this->declare_parameter(
        "pointcloud_topic", runtime_options.pointcloud_topic
    );
    runtime_options.frame_id = this->declare_parameter(
        "frame_id", runtime_options.frame_id
    );
    double angle_offset_deg = this->declare_parameter(
        "angle_offset_deg", runtime_options.angle_offset_rad * 180.0 / PI
    );
    runtime_options.angle_offset_rad = angle_offset_deg * PI / 180.0;
    runtime_options.safety_debug = this->declare_parameter(
        "safety_debug", runtime_options.safety_debug
    );
    runtime_options.http_host = this->declare_parameter(
        "http_host", runtime_options.http_host
    );
    runtime_options.http_port = this->declare_parameter(
        "http_port", runtime_options.http_port
    );
    runtime_options.http_timeout_ms = this->declare_parameter(
        "http_timeout_ms", runtime_options.http_timeout_ms
    );
    runtime_options.safety_zones_topic = this->declare_parameter(
        "safety_zones_topic", runtime_options.safety_zones_topic
    );
    runtime_options.safety_status_topic = this->declare_parameter(
        "safety_zones_status_topic", runtime_options.safety_status_topic
    );
    runtime_options.safety_debug_period_s = this->declare_parameter(
        "safety_debug_period_s", runtime_options.safety_debug_period_s
    );
    runtime_options.diagnostics_enabled = this->declare_parameter(
        "diagnostics_enabled", runtime_options.diagnostics_enabled
    );
    runtime_options.diagnostics_topic = this->declare_parameter(
        "diagnostics_topic", runtime_options.diagnostics_topic
    );
    runtime_options.diagnostics_period_s = this->declare_parameter(
        "diagnostics_period_s", runtime_options.diagnostics_period_s
    );
    runtime_options.no_data_timeout_s = this->declare_parameter(
        "no_data_timeout_s", runtime_options.no_data_timeout_s
    );
    runtime_options.udp_reconnect_initial_ms = this->declare_parameter(
        "udp_reconnect_initial_ms", runtime_options.udp_reconnect_initial_ms
    );
    runtime_options.udp_reconnect_max_ms = this->declare_parameter(
        "udp_reconnect_max_ms", runtime_options.udp_reconnect_max_ms
    );
    if (runtime_options.udp_reconnect_initial_ms <= 0 ||
        runtime_options.udp_reconnect_max_ms <
            runtime_options.udp_reconnect_initial_ms) {
        throw std::invalid_argument(
            "UDP reconnect delays must satisfy 0 < initial_ms <= max_ms"
        );
    }

    // setup socket for receiving data
    socket.reset(new receiver_socket(this->ip_addr, this->port));

    laserscan_publisher =
        this->create_publisher<sensor_msgs::msg::LaserScan>(cloud_topic, 128);
    pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_options.pointcloud_topic.empty()
            ? default_pointcloud_topic(cloud_topic)
            : runtime_options.pointcloud_topic,
        128
    );
    if (runtime_options.safety_debug) {
        safety_zones_publisher =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                runtime_options.safety_zones_topic, 1
            );
        safety_status_publisher =
            this->create_publisher<std_msgs::msg::String>(
                runtime_options.safety_status_topic, 1
            );
        safety_debug_running = true;
        safety_debug_thread = std::thread([this]() {
            while (rclcpp::ok() && safety_debug_running.load()) {
                publish_safety_debug();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }
    if (runtime_options.diagnostics_enabled) {
        diagnostics_publisher =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                runtime_options.diagnostics_topic, 1
            );
        diagnostics_running = true;
        diagnostics_thread = std::thread([this]() {
            while (rclcpp::ok() && diagnostics_running.load()) {
                publish_diagnostics();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

}

Driver::Driver(
    std::string pcap_path, std::string cloud_topic, bool is_full,
    DriverRuntimeOptions options
) : Node("driver"), pcap_path(pcap_path), is_full(is_full),
    runtime_options(std::move(options)),
    started_at(std::chrono::steady_clock::now()) {

    this->pcap_path = this->declare_parameter("pcap_path", this->pcap_path);
    cloud_topic = this->declare_parameter("scan_topic", cloud_topic);
    const std::string capture_type = this->declare_parameter(
        "capture_type", this->is_full ? "FULL" : "SINGLE"
    );
    if (capture_type != "FULL" && capture_type != "SINGLE") {
        throw std::invalid_argument("capture_type must be FULL or SINGLE");
    }
    this->is_full = capture_type == "FULL";

    runtime_options.pointcloud_topic = this->declare_parameter(
        "pointcloud_topic", runtime_options.pointcloud_topic
    );
    runtime_options.frame_id = this->declare_parameter(
        "frame_id", runtime_options.frame_id
    );
    double angle_offset_deg = this->declare_parameter(
        "angle_offset_deg", runtime_options.angle_offset_rad * 180.0 / PI
    );
    runtime_options.angle_offset_rad = angle_offset_deg * PI / 180.0;
    runtime_options.safety_debug = this->declare_parameter(
        "safety_debug", runtime_options.safety_debug
    );
    runtime_options.http_host = this->declare_parameter(
        "http_host", runtime_options.http_host
    );
    runtime_options.http_port = this->declare_parameter(
        "http_port", runtime_options.http_port
    );
    runtime_options.http_timeout_ms = this->declare_parameter(
        "http_timeout_ms", runtime_options.http_timeout_ms
    );
    runtime_options.safety_zones_topic = this->declare_parameter(
        "safety_zones_topic", runtime_options.safety_zones_topic
    );
    runtime_options.safety_status_topic = this->declare_parameter(
        "safety_zones_status_topic", runtime_options.safety_status_topic
    );
    runtime_options.safety_debug_period_s = this->declare_parameter(
        "safety_debug_period_s", runtime_options.safety_debug_period_s
    );
    runtime_options.diagnostics_enabled = this->declare_parameter(
        "diagnostics_enabled", runtime_options.diagnostics_enabled
    );
    runtime_options.diagnostics_topic = this->declare_parameter(
        "diagnostics_topic", runtime_options.diagnostics_topic
    );
    runtime_options.diagnostics_period_s = this->declare_parameter(
        "diagnostics_period_s", runtime_options.diagnostics_period_s
    );
    runtime_options.no_data_timeout_s = this->declare_parameter(
        "no_data_timeout_s", runtime_options.no_data_timeout_s
    );
    runtime_options.udp_reconnect_initial_ms = this->declare_parameter(
        "udp_reconnect_initial_ms", runtime_options.udp_reconnect_initial_ms
    );
    runtime_options.udp_reconnect_max_ms = this->declare_parameter(
        "udp_reconnect_max_ms", runtime_options.udp_reconnect_max_ms
    );
    if (runtime_options.udp_reconnect_initial_ms <= 0 ||
        runtime_options.udp_reconnect_max_ms <
            runtime_options.udp_reconnect_initial_ms) {
        throw std::invalid_argument(
            "UDP reconnect delays must satisfy 0 < initial_ms <= max_ms"
        );
    }

    // setup libtins sniffer for reading data
    pcap_sniffer.reset(new Tins::FileSniffer{this->pcap_path});

    // get the first packet's timestamp for time-correct packets reading
    Tins::Packet _pkt(pcap_sniffer->next_packet());
    _prev_pkt_tmstmp = _pkt.timestamp().microseconds();

    // ROS publising routine
    laserscan_publisher =
        this->create_publisher<sensor_msgs::msg::LaserScan>(cloud_topic, 128);
    pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_options.pointcloud_topic.empty()
            ? default_pointcloud_topic(cloud_topic)
            : runtime_options.pointcloud_topic,
        128
    );
    if (runtime_options.safety_debug) {
        safety_zones_publisher =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                runtime_options.safety_zones_topic, 1
            );
        safety_status_publisher =
            this->create_publisher<std_msgs::msg::String>(
                runtime_options.safety_status_topic, 1
            );
        safety_debug_running = true;
        safety_debug_thread = std::thread([this]() {
            while (rclcpp::ok() && safety_debug_running.load()) {
                publish_safety_debug();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }
    if (runtime_options.diagnostics_enabled) {
        diagnostics_publisher =
            this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                runtime_options.diagnostics_topic, 1
            );
        diagnostics_running = true;
        diagnostics_thread = std::thread([this]() {
            while (rclcpp::ok() && diagnostics_running.load()) {
                publish_diagnostics();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

}

Driver::~Driver() {
    safety_debug_running = false;
    diagnostics_running = false;
    if (safety_debug_thread.joinable()) {
        safety_debug_thread.join();
    }
    if (diagnostics_thread.joinable()) {
        diagnostics_thread.join();
    }
}

void Driver::poll() {

    // pcap_sniffer provided?
    if (pcap_sniffer)
        _poll_pcap();

    // UDP mode otherwise
    else
        _poll_udp();
}

void Driver::poll_full() {

    // pcap_sniffer provided?
    if (pcap_sniffer)
        _poll_full_pcap();

    // UDP mode otherwise
    else
        _poll_full_udp();
}

void Driver::publish_safety_debug() {
    if (!runtime_options.safety_debug) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    double period_s = std::max(0.1, runtime_options.safety_debug_period_s);
    if (last_safety_debug_publish.time_since_epoch().count() != 0) {
        auto elapsed = std::chrono::duration<double>(
            now - last_safety_debug_publish
        );
        if (elapsed.count() < period_s) {
            return;
        }
    }
    last_safety_debug_publish = now;

    try {
        HttpClient client(
            runtime_options.http_host, runtime_options.http_port,
            runtime_options.http_timeout_ms
        );

        json status_json = json::object();
        HttpResponse status_response = client.get("/zones/status.json");
        if (status_response.status_code >= 200 &&
            status_response.status_code < 300) {
            std_msgs::msg::String status_msg;
            status_msg.data = status_response.body;
            safety_status_publisher->publish(status_msg);
            if (!status_response.body.empty()) {
                status_json = json::parse(status_response.body);
            }
        }

        HttpResponse zones_response = client.get("/zones.json");
        if (zones_response.status_code >= 200 &&
            zones_response.status_code < 300 && !zones_response.body.empty()) {
            visualization_msgs::msg::MarkerArray markers =
                build_safety_zone_markers(
                    json::parse(zones_response.body), status_json,
                    runtime_options.frame_id, runtime_options.angle_offset_rad,
                    this->get_clock()->now()
                );
            safety_zones_publisher->publish(markers);
        }
    }
    catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Safety debug polling failed: %s", ex.what()
        );
    }
}

void Driver::record_published_message() {
    ++messages_published;
    last_publish_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

void Driver::record_socket_error(const std::string& message) {
    ++socket_errors;
    std::lock_guard<std::mutex> lock(socket_error_mutex);
    last_socket_error = message;
}

bool Driver::receive_udp_packet(uint8_t* buffer, int length) {
    while (rclcpp::ok()) {
        try {
            if (socket->get_packet(buffer, length) == 0) {
                return true;
            }
        }
        catch (const std::system_error& ex) {
            socket_recovering = true;
            record_socket_error(ex.what());
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "UDP socket failed; starting recovery: %s", ex.what()
            );

            int retry_delay_ms = std::max(
                1, runtime_options.udp_reconnect_initial_ms
            );
            const int max_delay_ms = std::max(
                retry_delay_ms, runtime_options.udp_reconnect_max_ms
            );
            while (rclcpp::ok()) {
                socket_retry_delay_ms = retry_delay_ms;
                int remaining_ms = retry_delay_ms;
                while (remaining_ms > 0 && rclcpp::ok()) {
                    const int step_ms = std::min(remaining_ms, 100);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(step_ms)
                    );
                    remaining_ms -= step_ms;
                }
                if (!rclcpp::ok()) {
                    return false;
                }

                try {
                    socket->reopen();
                    ++socket_reconnects;
                    socket_recovering = false;
                    socket_retry_delay_ms = 0;
                    RCLCPP_INFO(this->get_logger(), "UDP socket recovered");
                    break;
                }
                catch (const std::exception& reopen_error) {
                    record_socket_error(reopen_error.what());
                    RCLCPP_ERROR_THROTTLE(
                        this->get_logger(), *this->get_clock(), 5000,
                        "UDP socket reopen failed: %s", reopen_error.what()
                    );
                    retry_delay_ms += std::min(
                        retry_delay_ms, max_delay_ms - retry_delay_ms
                    );
                }
            }
        }
    }
    return false;
}

void Driver::publish_diagnostics() {
    const auto now = std::chrono::steady_clock::now();
    const double period_s = std::max(0.1, runtime_options.diagnostics_period_s);
    if (last_diagnostics_publish.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(
            now - last_diagnostics_publish
        ).count() < period_s) {
        return;
    }
    last_diagnostics_publish = now;

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = this->get_clock()->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = this->get_fully_qualified_name() + std::string("/data_stream");
    status.hardware_id = diagnostic_hardware_id(
        ip_addr, pcap_path, runtime_options.http_host
    );

    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()
    ).count();
    const int64_t published_ns = last_publish_ns.load();
    const double age_s = published_ns == 0
        ? std::chrono::duration<double>(now - started_at).count()
        : static_cast<double>(now_ns - published_ns) / 1.0e9;

    if (socket_recovering.load()) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "UDP socket recovering";
    }
    else if (age_s > runtime_options.no_data_timeout_s) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "No scan data";
    }
    else if (messages_published.load() == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Waiting for first scan";
    }
    else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Streaming";
    }

    auto add_value = [&](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
    };
    add_value("transport", runtime_options.transport);
    add_value("last_scan_age_s", std::to_string(age_s));
    add_value("packets_received", std::to_string(packets_received.load()));
    add_value("messages_published", std::to_string(messages_published.load()));
    add_value("invalid_packets", std::to_string(invalid_packets.load()));
    add_value(
        "discarded_revolutions",
        std::to_string(discarded_revolutions.load())
    );
    add_value(
        "socket_state",
        socket ? (socket_recovering.load() ? "recovering" : "ready")
               : "not_applicable"
    );
    add_value("socket_errors", std::to_string(socket_errors.load()));
    add_value("socket_reconnects", std::to_string(socket_reconnects.load()));
    add_value(
        "socket_retry_delay_ms", std::to_string(socket_retry_delay_ms.load())
    );
    {
        std::lock_guard<std::mutex> lock(socket_error_mutex);
        add_value("socket_last_error", last_socket_error);
    }
    message.status.push_back(status);
    diagnostics_publisher->publish(message);
}

void Driver::_poll_full_udp() {
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    FullScanAssembler assembler;

    while (rclcpp::ok() && !assembler.complete()) {
        packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);
        if (!receive_udp_packet(raw_pkt.get(), packet::PKT_LEN)) {
            return;
        }

        try {
            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));
            ++packets_received;
            assembler.add_packet(hdl_pkt);
        }
        catch (const std::exception& ex) {
            ++invalid_packets;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Invalid LiDAR packet: %s", ex.what()
            );
        }
    }

    msg->ranges.resize(POINTS_PER_REV);
    msg->intensities.resize(POINTS_PER_REV);
    for (size_t index = 0; index < POINTS_PER_REV; ++index) {
        msg->ranges[index] = assembler.ranges_mm()[index] / 1000.0F;
        msg->intensities[index] = assembler.intensities()[index];
    }

    msg->angle_min       = runtime_options.angle_offset_rad;
    msg->angle_max       = runtime_options.angle_offset_rad + 2 * PI;
    msg->angle_increment = 2 * PI / POINTS_PER_REV;
    msg->scan_time = assembler.rotation_frequency_hz() > 0
                         ? 1.0 / assembler.rotation_frequency_hz()
                         : 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    msg->range_min       = range_min_or_default(msg->ranges);
    msg->range_max       = range_max_or_default(msg->ranges);
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = runtime_options.frame_id;

    if (assembler.discarded_revolutions() > 0) {
        discarded_revolutions += assembler.discarded_revolutions();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Discarded %zu incomplete LiDAR revolution(s)",
            assembler.discarded_revolutions()
        );
    }

    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
    record_published_message();
}

void Driver::_poll_full_pcap() {
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    FullScanAssembler assembler;

    while (rclcpp::ok() && !assembler.complete()) {
        Tins::Packet pkt(pcap_sniffer->next_packet());
        if (!pkt) {
            RCLCPP_INFO(this->get_logger(), "Starting PCAP over");
            pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});
            _prev_pkt_tmstmp = 0;
            continue;
        }

        const auto current_timestamp = pkt.timestamp().microseconds();
        if (_prev_pkt_tmstmp > 0 && current_timestamp >= _prev_pkt_tmstmp) {
            std::this_thread::sleep_for(
                std::chrono::microseconds(current_timestamp - _prev_pkt_tmstmp)
            );
        }
        _prev_pkt_tmstmp = current_timestamp;

        std::vector<uint8_t> raw_pdu =
            pkt.pdu()->rfind_pdu<Tins::RawPDU>().payload();
        if (raw_pdu.size() != packet::PKT_LEN) {
            ++invalid_packets;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Ignoring PCAP payload with %zu bytes", raw_pdu.size()
            );
            continue;
        }

        packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);
        std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());
        try {
            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));
            ++packets_received;
            assembler.add_packet(hdl_pkt);
        }
        catch (const std::exception& ex) {
            ++invalid_packets;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Invalid LiDAR packet: %s", ex.what()
            );
        }
    }

    msg->ranges.resize(POINTS_PER_REV);
    msg->intensities.resize(POINTS_PER_REV);
    for (size_t index = 0; index < POINTS_PER_REV; ++index) {
        msg->ranges[index] = assembler.ranges_mm()[index] / 1000.0F;
        msg->intensities[index] = assembler.intensities()[index];
    }

    msg->angle_min       = runtime_options.angle_offset_rad;
    msg->angle_max       = runtime_options.angle_offset_rad + 2 * PI;
    msg->angle_increment = 2 * PI / POINTS_PER_REV;
    msg->scan_time = assembler.rotation_frequency_hz() > 0
                         ? 1.0 / assembler.rotation_frequency_hz()
                         : 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    msg->range_min       = range_min_or_default(msg->ranges);
    msg->range_max       = range_max_or_default(msg->ranges);
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = runtime_options.frame_id;

    if (assembler.discarded_revolutions() > 0) {
        discarded_revolutions += assembler.discarded_revolutions();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Discarded %zu incomplete LiDAR revolution(s)",
            assembler.discarded_revolutions()
        );
    }

    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
    record_published_message();
}

std::pair<std::string, unsigned> Driver::get_network_params() {
    return {ip_addr, port};
}

void Driver::_poll_udp() {

    // initialize ros pointcloud v2 message
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // initialize raw packet collection
    packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

    // wait until we are receive data
    if (!receive_udp_packet(raw_pkt.get(), packet::PKT_LEN)) {
        return;
    }

    std::unique_ptr<pkt_hdl_Mech> hdl_pkt;
    try {
        hdl_pkt = std::make_unique<pkt_hdl_Mech>(std::move(raw_pkt));
        ++packets_received;
    }
    catch (const std::exception& ex) {
        ++invalid_packets;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Invalid LiDAR packet: %s", ex.what()
        );
        return;
    }

    // fill ros message by data from the handled packet
    msg->angle_min       = hdl_pkt->angles[0] + runtime_options.angle_offset_rad;
    msg->angle_max       = hdl_pkt->angles[hdl_pkt->CHANELLS - 1] +
                           runtime_options.angle_offset_rad;
    msg->angle_increment = hdl_pkt->RAD_RESOLUTION;
    msg->scan_time = hdl_pkt->rotation_frequency_hz() > 0
                         ? 1.0 / hdl_pkt->rotation_frequency_hz()
                         : 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    for (size_t chnl = 0; chnl < hdl_pkt->CHANELLS; ++chnl) {
        msg->ranges.push_back(hdl_pkt->ranges[chnl] / 1000);
        msg->intensities.push_back(hdl_pkt->intensities[chnl]);
    }
    msg->range_min = range_min_or_default(msg->ranges);
    msg->range_max = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = runtime_options.frame_id;

    // publish ros messages to topics
    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
    record_published_message();
}

void Driver::_poll_pcap() {

    // initialzie ros pointcloud v2 message
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    // get the next packet from the target PCAP file
    Tins::Packet pkt(pcap_sniffer->next_packet());

    // is packet extracted with problems?
    if (!pkt) {
        RCLCPP_INFO(this->get_logger(), "Starting PCAP over");
        pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});
        Tins::Packet _pkt(pcap_sniffer->next_packet());
        _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
        return;
    }

    // normal operation otherwise
    else {

        // sleep for time-correct packets reading
        auto _cur_pkt_tmstmp = pkt.timestamp().microseconds();
        std::this_thread::sleep_for(
            std::chrono::microseconds(_cur_pkt_tmstmp - _prev_pkt_tmstmp)
        );
        _prev_pkt_tmstmp = _cur_pkt_tmstmp;

        // create raw-pdu collection
        std::vector<uint8_t> raw_pdu =
            pkt.pdu()->rfind_pdu<Tins::RawPDU>().payload();
        if (raw_pdu.size() != packet::PKT_LEN) {
            ++invalid_packets;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Ignoring PCAP payload with %zu bytes", raw_pdu.size()
            );
            return;
        }

        // initialize raw packet collection
        packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]);

        // fill raw_pkt with raw-pdu
        std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

        // transform raw packet to handled packet
        std::unique_ptr<pkt_hdl_Mech> hdl_pkt;
        try {
            hdl_pkt = std::make_unique<pkt_hdl_Mech>(std::move(raw_pkt));
            ++packets_received;
        }
        catch (const std::exception& ex) {
            ++invalid_packets;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Invalid LiDAR packet: %s", ex.what()
            );
            return;
        }

        // fill ros message by data from the handled packet
        msg->angle_min = hdl_pkt->angles[0] + runtime_options.angle_offset_rad;
        msg->angle_max = hdl_pkt->angles[hdl_pkt->CHANELLS - 1] +
                         runtime_options.angle_offset_rad;
        msg->angle_increment = hdl_pkt->RAD_RESOLUTION;
        msg->scan_time = hdl_pkt->rotation_frequency_hz() > 0
                             ? 1.0 / hdl_pkt->rotation_frequency_hz()
                             : 0.1;
        msg->time_increment  = msg->scan_time / POINTS_PER_REV;
        for (size_t chnl = 0; chnl < hdl_pkt->CHANELLS; ++chnl) {
            msg->ranges.push_back(hdl_pkt->ranges[chnl] / 1000);
            msg->intensities.push_back(hdl_pkt->intensities[chnl]);
        }
        msg->range_min = range_min_or_default(msg->ranges);
        msg->range_max = range_max_or_default(msg->ranges);
    }

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = runtime_options.frame_id;

    // publish ros messages to topics
    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
    record_published_message();
}
} // namespace dephan_ros
