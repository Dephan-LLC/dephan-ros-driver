/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.cpp
 * @brief ROS driver for DEPHAN LLC LiDars
 */

#include "ros_driver.hpp"
#include <vector>

namespace dephan_ros {
using namespace std::chrono_literals;

Driver::Driver(
    std::string ip_addr, unsigned port, std::string cloud_topic, bool is_full
) : Node("driver"), ip_addr(ip_addr), port(port), is_full(is_full) {

    // setup socket for receiving data
    socket.reset(new receiver_socket(ip_addr, port));

    laserscan_publisher =
        this->create_publisher<sensor_msgs::msg::LaserScan>(cloud_topic, 128);

    timer =
        this->create_wall_timer(1ms, std::bind(&Driver::timer_callback, this));
}

Driver::Driver(std::string pcap_path, std::string cloud_topic, bool is_full) :
    Node("driver"), pcap_path(pcap_path), is_full(is_full) {

    // setup libtins sniffer for reading data
    pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});

    // get the first packet's timestamp for time-correct packets reading
    Tins::Packet _pkt(pcap_sniffer->next_packet());
    _prev_pkt_tmstmp = _pkt.timestamp().microseconds();

    // ROS publising routine
    laserscan_publisher =
        this->create_publisher<sensor_msgs::msg::LaserScan>(cloud_topic, 128);

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

    // wait until 18 packeges are recieved
    for (size_t i = 0; i < 18; i++) {

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
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
            msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
            msg->intensities.push_back(hdl_pkt.intensities[chnl]);
        }
    }
    // fill ros message by constant data
    msg->angle_min       = 0.0;
    msg->angle_max       = 2 * 3.1415;
    msg->angle_increment = 2 * 3.1415 / 2300;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / 2300.0;
    msg->range_min = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    msg->range_max = *std::max_element(msg->ranges.begin(), msg->ranges.end());

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros message to topic
    laserscan_publisher->publish(*msg);
}

void Driver::_poll_full_pcap() {

    // initialzie ros message
    sensor_msgs::msg::LaserScan::Ptr msg(new sensor_msgs::msg::LaserScan);

    // wait until 18 packages are readed from the target PCAP file
    for (size_t i = 0; i < 18; i++) {

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
            for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
                msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
                msg->intensities.push_back(hdl_pkt.intensities[chnl]);
            }
        }
    }
    // fill ros message by constant data
    msg->angle_min       = 0.0;
    msg->angle_max       = 2 * 3.1415;
    msg->angle_increment = 2 * 3.1415 / 2300;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / 2300.0;
    msg->range_min = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    msg->range_max = *std::max_element(msg->ranges.begin(), msg->ranges.end());

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros message to topic
    laserscan_publisher->publish(*msg);
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
    msg->time_increment  = msg->scan_time / 2300.0;
    for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
        msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
        msg->intensities.push_back(hdl_pkt.intensities[chnl]);
    }
    msg->range_min = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    msg->range_max = *std::max_element(msg->ranges.begin(), msg->ranges.end());

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros message to topic
    laserscan_publisher->publish(*msg);
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
        msg->time_increment  = msg->scan_time / 2300.0;
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
            msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
            msg->intensities.push_back(hdl_pkt.intensities[chnl]);
        }
        msg->range_min =
            *std::min_element(msg->ranges.begin(), msg->ranges.end());
        msg->range_max =
            *std::max_element(msg->ranges.begin(), msg->ranges.end());
    }

    // add timestamp to ros message
    msg->header.stamp = this->get_clock()->now();

    // // add frame id to ros message
    msg->header.frame_id = "map";

    // publish ros message to topic
    laserscan_publisher->publish(*msg);
}
} // namespace dephan_ros
