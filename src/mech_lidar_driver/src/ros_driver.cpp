/** 
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.cpp
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

        // setup socket for receiving data
        socket.reset(new receiver_socket(ip_addr, port));

        // ROS publising routine
        rawdata_publihser = 
            nh.advertise<std_msgs::UInt8MultiArray>("raw_data", 10);
        pointcloud_publisher = 
            nh.advertise<sensor_msgs::PointCloud>("point_cloud_data", 10); 
        pointcloud2_publisher = 
            nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(cloud_topic, 10); 
    }

    Driver::Driver(ros::NodeHandle nh, std::string pcap_path, std::string cloud_topic): 
        pcap_path(pcap_path) {
        
        // setup libtins sniffer for reading data
        pcap_sniffer.reset(new Tins::FileSniffer {pcap_path});

        // get the first packet's timestamp for time-correct packets reading 
        Tins::Packet _pkt(pcap_sniffer->next_packet());
        _prev_pkt_tmstmp = _pkt.timestamp().microseconds();

        // ROS publising routine
        pointcloud2_publisher = 
            nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(cloud_topic, 10); 
    }

    void 
    Driver::poll() {
        // pcap_sniffer provided?
        if (pcap_sniffer)
            _poll_pcap(); 
        // UDP mode otherwise
        else    
            _poll_udp();
    }

    void 
    Driver::poll_full() {
        // pcap_sniffer provided? 
        if (pcap_sniffer)
            _poll_full_pcap();
        // UDP mode otherwise
        else
            _poll_full_udp();
    }

    void 
    Driver::_poll_full_udp() {
        // initialzie ros pointcloud v2 message
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        // wait until 18 packeges are recieved
        for (size_t i = 0; i < 18; i++) { 
            // initialize raw packet collection
            packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

            // wait until we are receive data
            while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)); 

            // transform raw packet to handled packet
            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

            // fill ros message by data from the handled packet
            for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) 
                msg->points.push_back(
                    pcl::PointXYZ(
                        hdl_pkt.ranges[chnl] * cosf(hdl_pkt.angles[chnl]) / 1000, 
                        hdl_pkt.ranges[chnl] * sinf(hdl_pkt.angles[chnl]) / 1000, 
                        0.0
                    )
                );
        }
        // add timestamp to ros message
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        // add frame id to ros message
        msg->header.frame_id = "map";

        // publish ros message to topic
        pointcloud2_publisher.publish(msg);
    }

    void 
    Driver::_poll_full_pcap() {
        // initialzie ros pointcloud v2 message
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
        // initialize ros pointcloud v2 message
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        // initialize raw packet collection
        packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

        // wait until we are receive data
        while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)); 

        // transform raw packet to handled packet
        pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

        // fill ros message by data from the handled packet
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) 
            msg->points.push_back(
                pcl::PointXYZ(
                    hdl_pkt.ranges[chnl] * cosf(hdl_pkt.angles[chnl]) / 1000, 
                    hdl_pkt.ranges[chnl] * sinf(hdl_pkt.angles[chnl]) / 1000, 
                    0.0
                )
            );
        
        // add timestamp to ros message
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        // add frame id to ros message
        msg->header.frame_id = "map";

        // publish ros message to topic
        pointcloud2_publisher.publish(msg);
    }

    void 
    Driver::_poll_pcap() {
        // initialzie ros pointcloud v2 message
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        // get the next packet from the target PCAP file
        Tins::Packet pkt(pcap_sniffer->next_packet());

        // is packet extracted with problems?
        if (!pkt) {
            ROS_INFO("Starting over...");
            pcap_sniffer.reset(new Tins::FileSniffer {pcap_path});
            Tins::Packet _pkt(pcap_sniffer->next_packet());
            _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
        }
        // normal operation otherwise
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

            // fill ros message by data from the handled packet
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

        // add frame id to ros message
        msg->header.frame_id = "map";

        // publish ros message to topic
        pointcloud2_publisher.publish(msg);
    }
}