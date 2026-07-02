/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.cpp
 * @brief ROS driver for DEPHAN LLC LiDars
 */

#include "ros_driver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

namespace dephan_ros {
using namespace std::chrono_literals;
namespace {
const float PI = 3.14159265358979323846f;
const int POINTS_PER_REV = pkt_hdl_Mech::POINTS_PER_REV;
const int PACKETS_PER_REV = POINTS_PER_REV / pkt_hdl_Mech::CHANELLS;

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
    std::string pointcloud_topic
) : Node("driver"), ip_addr(ip_addr), port(port), is_full(is_full) {

    // setup socket for receiving data
    socket.reset(new receiver_socket(ip_addr, port));

    laserscan_publisher =
        this->create_publisher<sensor_msgs::msg::LaserScan>(cloud_topic, 128);
    pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic.empty() ? default_pointcloud_topic(cloud_topic)
                                 : pointcloud_topic,
        128
    );

    timer =
        this->create_wall_timer(1ms, std::bind(&Driver::timer_callback, this));
}

Driver::Driver(
    std::string pcap_path, std::string cloud_topic, bool is_full,
    std::string pointcloud_topic
) : Node("driver"), pcap_path(pcap_path), is_full(is_full) {

    // setup libtins sniffer for reading data
    pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});

    // get the first packet's timestamp for time-correct packets reading
    Tins::Packet _pkt(pcap_sniffer->next_packet());
    _prev_pkt_tmstmp = _pkt.timestamp().microseconds();

    // ROS publising routine
    laserscan_publisher =
        this->create_publisher<sensor_msgs::msg::LaserScan>(cloud_topic, 128);
    pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic.empty() ? default_pointcloud_topic(cloud_topic)
                                 : pointcloud_topic,
        128
    );

    timer =
        this->create_wall_timer(1ms, std::bind(&Driver::timer_callback, this));
}

void Driver::timer_callback() {
    if (is_full)
        poll_full();
    else
        poll();
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

void Driver::_poll_full_udp() {

    // initialzie ros pointcloud v2 message
    sensor_msgs::msg::LaserScan::Ptr msg(new sensor_msgs::msg::LaserScan);

    msg->ranges.assign(POINTS_PER_REV, std::numeric_limits<float>::infinity());
    msg->intensities.assign(POINTS_PER_REV, 0.0f);

    // wait until one full revolution is received
    for (size_t i = 0; i < PACKETS_PER_REV; i++) {

        // initialize raw packet collection
        packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

        // wait until we are receive data
        while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)) {
            if (!rclcpp::ok()) {
                return;
            }
        }

        // transform raw packet to handled packet
        pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

        // fill ros message by encoder point index
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
            uint16_t point_idx = hdl_pkt.point_index(chnl);
            msg->ranges[point_idx] = hdl_pkt.ranges[chnl] / 1000;
            msg->intensities[point_idx] = hdl_pkt.intensities[chnl];
        }
    }
    // fill ros message by constant data
    msg->angle_min       = 0.0;
    msg->angle_max       = 2 * PI;
    msg->angle_increment = 2 * PI / POINTS_PER_REV;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    msg->range_min       = range_min_or_default(msg->ranges);
    msg->range_max       = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros messages to topics
    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
}

void Driver::_poll_full_pcap() {

    // initialzie ros message
    sensor_msgs::msg::LaserScan::Ptr msg(new sensor_msgs::msg::LaserScan);

    msg->ranges.assign(POINTS_PER_REV, std::numeric_limits<float>::infinity());
    msg->intensities.assign(POINTS_PER_REV, 0.0f);

    // wait until one full revolution is read from the target PCAP file
    for (size_t i = 0; i < PACKETS_PER_REV; i++) {

        // get the next packet from the target PCAP file
        Tins::Packet pkt(pcap_sniffer->next_packet());

        // is packet extracted with problems?
        if (!pkt) {
            // ROS_INFO("Starting over...");
            std::cout << "Starting over..." << std::endl;
            pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});
            Tins::Packet _pkt(pcap_sniffer->next_packet());
            _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
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

            // initialize raw packet collection
            packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]);

            // fill raw_pkt with raw-pdu
            std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

            // transform raw packet to handled packet
            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

            // fill ros message by encoder point index
            for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
                uint16_t point_idx = hdl_pkt.point_index(chnl);
                msg->ranges[point_idx] = hdl_pkt.ranges[chnl] / 1000;
                msg->intensities[point_idx] = hdl_pkt.intensities[chnl];
            }
        }
    }
    // fill ros message by constant data
    msg->angle_min       = 0.0;
    msg->angle_max       = 2 * PI;
    msg->angle_increment = 2 * PI / POINTS_PER_REV;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    msg->range_min       = range_min_or_default(msg->ranges);
    msg->range_max       = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros messages to topics
    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
}

std::pair<std::string, unsigned> Driver::get_network_params() {
    return {ip_addr, port};
}

void Driver::_poll_udp() {

    // initialize ros pointcloud v2 message
    sensor_msgs::msg::LaserScan::Ptr msg(new sensor_msgs::msg::LaserScan);

    // initialize raw packet collection
    packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

    // wait until we are receive data
    while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)) {
        if (!rclcpp::ok()) {
            return;
        }
    }

    // transform raw packet to handled packet
    pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

    // fill ros message by data from the handled packet
    msg->angle_min       = hdl_pkt.angles[0];
    msg->angle_max       = hdl_pkt.angles[hdl_pkt.CHANELLS - 1];
    msg->angle_increment = hdl_pkt.RAD_RESOLUTION;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
        msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
        msg->intensities.push_back(hdl_pkt.intensities[chnl]);
    }
    msg->range_min = range_min_or_default(msg->ranges);
    msg->range_max = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros messages to topics
    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
}

void Driver::_poll_pcap() {

    // initialzie ros pointcloud v2 message
    sensor_msgs::msg::LaserScan::Ptr msg(new sensor_msgs::msg::LaserScan);

    // get the next packet from the target PCAP file
    Tins::Packet pkt(pcap_sniffer->next_packet());

    // is packet extracted with problems?
    if (!pkt) {
        // ROS_INFO("Starting over...");
        std::cout << "Starting over..." << std::endl;
        pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});
        Tins::Packet _pkt(pcap_sniffer->next_packet());
        _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
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

        // initialize raw packet collection
        packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]);

        // fill raw_pkt with raw-pdu
        std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

        // transform raw packet to handled packet
        pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

        // fill ros message by data from the handled packet
        msg->angle_min       = hdl_pkt.angles[0];
        msg->angle_max       = hdl_pkt.angles[hdl_pkt.CHANELLS - 1];
        msg->angle_increment = hdl_pkt.RAD_RESOLUTION;
        msg->scan_time       = 0.1;
        msg->time_increment  = msg->scan_time / POINTS_PER_REV;
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
            msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
            msg->intensities.push_back(hdl_pkt.intensities[chnl]);
        }
        msg->range_min = range_min_or_default(msg->ranges);
        msg->range_max = range_max_or_default(msg->ranges);
    }

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros messages to topics
    sensor_msgs::msg::PointCloud2 cloud = make_pointcloud_msg(*msg);
    laserscan_publisher->publish(*msg);
    pointcloud_publisher->publish(cloud);
}
} // namespace dephan_ros
