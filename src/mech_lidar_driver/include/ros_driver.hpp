/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.hpp
 * @brief ROS driver for DEPHAN LLC LiDars
 */

#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tins/tins.h>
#include <thread>
#include <chrono>
#include <functional>

#include "packet_raw.hpp"
#include "reciever_socket.hpp"
#include "packet_handler_mech.hpp"

namespace dephan_ros {
/**
 * Runtime ROS publication and debug options.
 */
struct DriverRuntimeOptions {
    /**
     * Topic for PointCloud2 data derived from LaserScan ranges.
     */
    std::string pointcloud_topic;

    /**
     * Frame id written to LaserScan, PointCloud2 and safety debug messages.
     */
    std::string frame_id = "base_link";

    /**
     * Angle offset in radians applied to scan angles and zone visualization.
     */
    double angle_offset_rad = 1.5707963267948966;

    /**
     * Enables optional safety zone marker and status topic publication.
     */
    bool safety_debug = false;

    /**
     * HTTP endpoint used by safety debug polling.
     */
    std::string http_host = "192.168.0.120";
    int http_port         = 80;
    int http_timeout_ms   = 3000;

    /**
     * Safety debug topic names and polling period.
     */
    std::string safety_zones_topic  = "safety_zones_markers";
    std::string safety_status_topic = "safety_zones_status";
    double safety_debug_period_s    = 1.0;

    /** Standard diagnostics topic and no-data monitoring thresholds. */
    bool diagnostics_enabled = true;
    std::string diagnostics_topic = "diagnostics";
    double diagnostics_period_s = 1.0;
    double no_data_timeout_s = 2.0;
    int udp_reconnect_initial_ms = 250;
    int udp_reconnect_max_ms = 5000;
    std::string transport;
};

/**
 * Class for ROS driver.
 */
class Driver : public rclcpp::Node {
private:
    /**
     * IP address of the LiDar device.
     */
    std::string ip_addr;

    /**
     * Port of the LiDar device.
     */
    unsigned port;

    /**
     * Socket pointer for recieving data.
     */
    std::unique_ptr<receiver_socket> socket;

    /**
     * Path to pcap file that will be readed by driver.
     */
    std::string pcap_path;

    /**
     * Pointer in libtins' shiffer
     */
    std::unique_ptr<Tins::FileSniffer> pcap_sniffer;

    /**
     * Stored packet's timestamp for time correct reading from the pcap
     * file.
     */
    long long _prev_pkt_tmstmp;

    /**
     * Flag for LiDar angle (true for 2 pi rad segment per packet, false
     * for 2 pi / 18 rad segment per packet)
     */
    bool is_full = false;

    /**
     * Ros topic publisher for the ros laserscan data.
     */
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        laserscan_publisher;

    /**
     * Ros topic publisher for point cloud data derived from LaserScan points.
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud_publisher;

    /**
     * MarkerArray publisher for optional RViz safety zone visualization.
     */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        safety_zones_publisher;

    /**
     * String publisher with raw /zones/status.json data.
     */
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        safety_status_publisher;

    /** Standard DiagnosticArray publisher for stream health. */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostics_publisher;

    /**
     * Runtime ROS and optional debug settings.
     */
    DriverRuntimeOptions runtime_options;

    /**
     * Last wall-clock time when safety debug data was polled over HTTP.
     */
    std::chrono::steady_clock::time_point last_safety_debug_publish;

    /**
     * Background worker for safety debug polling.
     */
    std::atomic_bool safety_debug_running{false};
    std::thread safety_debug_thread;

    /** Background diagnostics state and stream counters. */
    std::atomic_bool diagnostics_running{false};
    std::thread diagnostics_thread;
    std::chrono::steady_clock::time_point started_at;
    std::chrono::steady_clock::time_point last_diagnostics_publish;
    std::atomic<int64_t> last_publish_ns{0};
    std::atomic<uint64_t> packets_received{0};
    std::atomic<uint64_t> messages_published{0};
    std::atomic<uint64_t> invalid_packets{0};
    std::atomic<uint64_t> discarded_revolutions{0};
    std::atomic_bool socket_recovering{false};
    std::atomic<uint64_t> socket_errors{0};
    std::atomic<uint64_t> socket_reconnects{0};
    std::atomic<int> socket_retry_delay_ms{0};
    mutable std::mutex socket_error_mutex;
    std::string last_socket_error;

    /**
     * Publish safety zone markers and status JSON when debug mode is enabled.
     */
    void publish_safety_debug();

    /** Publish current stream counters and no-data health state. */
    void publish_diagnostics();

    /** Record one successfully published LaserScan/PointCloud2 pair. */
    void record_published_message();

    /** Receive a UDP packet and recreate the socket after system errors. */
    bool receive_udp_packet(uint8_t* buffer, int length);

    /** Store an error for diagnostics without racing the diagnostics thread. */
    void record_socket_error(const std::string& message);

    /**
     * Poll one 115-point packet in UDP mode.
     */
    void _poll_udp();

    /**
     * Poll one 115-point packet in PCAP mode.
     */
    void _poll_pcap();

    /**
     * Poll the 20 packets of one complete revolution in UDP mode.
     */
    void _poll_full_udp();

    /**
     * Poll the 20 packets of one complete revolution in PCAP mode.
     */
    void _poll_full_pcap();

public:
    /**
     * Stops optional background safety debug polling.
     */
    ~Driver();

    /**
     * Driver UDP mode constructor.
     *
     * @param[in] ip_addr Ip address of the LiDar device.
     * @param[in] port Network port of the LiDar device.
     * @param[in] topic_name Name of the LaserScan topic generated by the Driver.
     * @param[in] is_full Flag for LiDar angle (true for 2 pi rad segment
     * per packet, false for 2 pi / 18 rad segment per packet).
     * @param[in] runtime_options ROS publication and optional debug settings.
     */
    Driver(
        std::string ip_addr, unsigned port, std::string topic_name,
        bool is_full = false,
        DriverRuntimeOptions runtime_options = DriverRuntimeOptions()
    );

    /**
     * Driver PCAP mode construcror.
     *
     * @param[in] pcap_path Path to the target PCAP file.
     * @param[in] topic_name Name of the LaserScan topic generated by the Driver.
     * @param[in] is_full Flag for LiDar angle (true for 2 pi rad segment
     * per packet, false for 2 pi / 18 rad segment per packet).
     * @param[in] runtime_options ROS publication and optional debug settings.
     */
    Driver(
        std::string pcap_path, std::string topic_name, bool is_full = false,
        DriverRuntimeOptions runtime_options = DriverRuntimeOptions()
    );

    /**
     * Polling function for the one packet from the LiDar.
     */
    void poll();

    /**
     * Polling function for the full revolution from the LiDar.
     */
    void poll_full();

    /**
     * Getter for the network parameters of the LiDar device.
     */
    std::pair<std::string, unsigned> get_network_params();
};
} // namespace dephan_ros

#endif
