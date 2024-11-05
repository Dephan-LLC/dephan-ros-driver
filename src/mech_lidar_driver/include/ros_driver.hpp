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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
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
     * Stored packet's timestamp for time correct reading from the pcap file.
     */
    long long _prev_pkt_tmstmp;

    /**
     * Flag for LiDar angle (true for 2 pi rad segment per packet, false for 2 pi / 18 rad segment per packet)
     */
    bool is_full = false;

    /**
     * Ros topic publisher for the ros pointcloud v2 data.
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud2_publisher;

    /**
     * Ros timer for polling operation.
     */
    rclcpp::TimerBase::SharedPtr timer;

    /**
     * Timer callback for polling operation.
     */
    void timer_callback();

    /**
     * Poll one packet (2 pi / 18 rad segment per packet) in UDP mode.
     */
    void _poll_udp();

    /**
     * Poll one packet (2 pi / 18 rad segment per packet) in PCAP mode.
     */
    void _poll_pcap();

    /**
     * Poll 18 packets (2 pi rad segment per packet) in UPD mode.
     */
    void _poll_full_udp();

    /**
     * Pill 18 packets (2 pi rad segment per packet) in PCAP mode.
     */
    void _poll_full_pcap();

public:
    /**
     * Driver UDP mode constructor.
     *
     * @param[in] ip_addr Ip address of the LiDar device.
     * @param[in] port Network port of the LiDar device.
     * @param[in] topic_name Name of the topic for pointcloud v2 messages
     * @param[in] is_full Flag for LiDar angle (true for 2 pi rad segment per packet, false for 2 pi / 18 rad segment per packet).
     */
    Driver(
        std::string ip_addr, unsigned port, std::string topic_name,
        bool is_full = false
    );

    /**
     * Driver PCAP mode construcror.
     *
     * @param[in] pcap_path Path to the target PCAP file.
     * @param[in] topic_name Name of the topic for pointcloud v2 messages.
     * @param[in] is_full Flag for LiDar angle (true for 2 pi rad segment per packet, false for 2 pi / 18 rad segment per packet).
     */
    Driver(std::string pcap_path, std::string topic_name, bool is_full = false);

    /**
     * Polling function for the one packet from the LiDar.
     */
    void poll();

    /**
     * Polling function for the 18 packets from the LiDar.
     */
    void poll_full();

    /**
     * Getter for the network parameters of the LiDar device.
     */
    std::pair<std::string, unsigned> get_network_params();
};
} // namespace dephan_ros

#endif
