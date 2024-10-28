/** 
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.hpp
 * @brief ROS driver for DEPHAN LLC LiDars
 */

#include "ros_driver.hpp"
#include <std_msgs/UInt8MultiArray.h>
#include <vector>


namespace dephan_ros { 
    Driver::Driver(ros::NodeHandle nh, std::string ip_addr, unsigned port, std::string cloud_topic):
        ip_addr(ip_addr), port(port) {
        // socket.reset(new receiver_socket("192.168.0.101", 9101));
        // socket.reset(new receiver_socket("192.168.0.120", 51551));
        socket.reset(new receiver_socket(ip_addr, port));

        rawdata_publihser = 
            nh.advertise<std_msgs::UInt8MultiArray>("raw_data", 10);
        pointcloud_publisher = 
            nh.advertise<sensor_msgs::PointCloud>("point_cloud_data", 10); 
        pointcloud2_publisher = 
            nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(cloud_topic, 10); 
    }

    Driver::Driver(ros::NodeHandle nh, std::string pcap_path, std::string cloud_topic): pcap_path(pcap_path) {
        pcap_sniffer.reset(new Tins::FileSniffer {pcap_path});

        Tins::Packet _pkt(pcap_sniffer->next_packet());
        _prev_pkt_tmstmp = _pkt.timestamp().microseconds();

        pointcloud2_publisher = 
            nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(cloud_topic, 10); 
    }

    void 
    Driver::poll() {
        if (pcap_sniffer)
            _poll_pcap(); 
        else    
            _poll_udp();
    }

    void 
    Driver::poll_full() {
        if (pcap_sniffer)
            _poll_full_pcap();
        else
            _poll_full_udp();
    }

    void 
    Driver::_poll_full_udp() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < 18; i++) { 
            packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

            while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)); 

            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

            for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) 
                msg->points.push_back(
                    pcl::PointXYZ(
                        hdl_pkt.ranges[chnl] * cosf(hdl_pkt.angles[chnl]) / 1000, 
                        hdl_pkt.ranges[chnl] * sinf(hdl_pkt.angles[chnl]) / 1000, 
                        0.0
                    )
                );
        }
        
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "map";

        pointcloud2_publisher.publish(msg);
    }

    void 
    Driver::_poll_full_pcap() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < 18; i++) {
            Tins::Packet pkt(pcap_sniffer->next_packet());

            if (!pkt) {
                ROS_INFO("Starting over...");
                pcap_sniffer.reset(new Tins::FileSniffer {pcap_path});
                Tins::Packet _pkt(pcap_sniffer->next_packet());
                _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
            }

            else {
                auto _cur_pkt_tmstmp = pkt.timestamp().microseconds();
                std::this_thread::sleep_for(
                    std::chrono::microseconds(_cur_pkt_tmstmp - _prev_pkt_tmstmp)
                );
                _prev_pkt_tmstmp = _cur_pkt_tmstmp;

                std::vector<uint8_t> raw_pdu = pkt.pdu()->rfind_pdu<Tins::RawPDU>().payload();                
                packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]); 
                std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

                // // // // 

                pkt_hdl_Mech hdl_pkt(std::move(raw_pkt)); 

                for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) 
                    msg->points.push_back(
                        pcl::PointXYZ(
                            hdl_pkt.ranges[chnl] * cosf(hdl_pkt.angles[chnl]) / 1000, 
                            hdl_pkt.ranges[chnl] * sinf(hdl_pkt.angles[chnl]) / 1000, 
                            0.0
                        )
                    );
            }
        }
        
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "map";

        pointcloud2_publisher.publish(msg);
    }

    std::pair<std::string, unsigned> 
    Driver::get_network_params() {
        return {ip_addr, port};
    }

    void 
    Driver::_poll_udp() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

        while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)); 

        pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) 
            msg->points.push_back(
                pcl::PointXYZ(
                    hdl_pkt.ranges[chnl] * cosf(hdl_pkt.angles[chnl]) / 1000, 
                    hdl_pkt.ranges[chnl] * sinf(hdl_pkt.angles[chnl]) / 1000, 
                    0.0
                )
            );
        
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "map";

        pointcloud2_publisher.publish(msg);
    }

    void 
    Driver::_poll_pcap() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        Tins::Packet pkt(pcap_sniffer->next_packet());

        if (!pkt) {
            ROS_INFO("Starting over...");
            pcap_sniffer.reset(new Tins::FileSniffer {pcap_path});
            Tins::Packet _pkt(pcap_sniffer->next_packet());
            _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
        }

        else {
            auto _cur_pkt_tmstmp = pkt.timestamp().microseconds();
            std::this_thread::sleep_for(
                std::chrono::microseconds(_cur_pkt_tmstmp - _prev_pkt_tmstmp)
            );
            _prev_pkt_tmstmp = _cur_pkt_tmstmp;

            std::vector<uint8_t> raw_pdu = pkt.pdu()->rfind_pdu<Tins::RawPDU>().payload();
        
            packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]); 
            std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt)); 

            for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) 
                msg->points.push_back(
                    pcl::PointXYZ(
                        hdl_pkt.ranges[chnl] * cosf(hdl_pkt.angles[chnl]) / 1000, 
                        hdl_pkt.ranges[chnl] * sinf(hdl_pkt.angles[chnl]) / 1000, 
                        0.0
                    )
                );
        }

        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->header.frame_id = "map";

        pointcloud2_publisher.publish(msg);
    }
}