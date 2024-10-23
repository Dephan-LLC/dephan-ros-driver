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

        // setup ROS publising routine
        rawdata_publihser = 
            nh.advertise<std_msgs::UInt8MultiArray>("raw_data", 10);
        pointcloud_publisher = 
            nh.advertise<sensor_msgs::PointCloud>("point_cloud_data", 10); 
        pointcloud2_publisher = 
            nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(cloud_topic, 10); 
    }

    void 
    Driver::poll() {
        // pcl pointcloud collection as a ros message
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
    Driver::poll_full() { 
        // pcl pointcloud collection as a ros message
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);

        // collect all 18 packets for 2 Pi scan picture
        for (size_t i = 0; i < 18; ++i) {
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
    
    std::pair<std::string, unsigned> 
    Driver::get_network_params() {
        return {ip_addr, port};
    }
}