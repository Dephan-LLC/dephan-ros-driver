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

#include <ros/ros.h>
#include <memory>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt8MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <tins/tins.h>
#include <thread>
#include <chrono>

#include "packet_raw.hpp"
#include "reciever_socket.hpp"
#include "packet_handler_mech.hpp"


namespace dephan_ros { 
    /**
     * Base class for driver.
     */
    class Driver {
    private:
        std::string ip_addr;    
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

        std::string pcap_path;
        std::unique_ptr<Tins::FileSniffer> pcap_sniffer;
        long long _prev_pkt_tmstmp;

        /**
         * Ros topic publisher for the rawdata.
         */
        ros::Publisher rawdata_publihser; 

        /**
         * Ros topic publisher for the ros pointcloud v1 data.
         */
        ros::Publisher pointcloud_publisher; 

        /**
         * Ros topic publisher for the ros pointcloud v2 data.
         */
        ros::Publisher pointcloud2_publisher; 

        void 
        _poll_udp(); 

        void 
        _poll_pcap();

        void 
        _poll_full_udp();

        void 
        _poll_full_pcap();
        
    public:
        /**
         * Driver constructor.
         * 
         * @param[in] nh Ros handle node.
         * @param[in] ip_addr Ip address of the LiDar device.
         * @param[in] port Network port of the LiDar device. 
         * @param[in] topic_name Name of the topic for pointcloud v2 messages.
         */
        Driver(ros::NodeHandle nh, std::string ip_addr, unsigned port, std::string topic_name); 
        Driver(ros::NodeHandle nh, std::string pcap_path, std::string topic_name);

        /**
         * Polling function for the one packet from the LiDar.
         */
        void 
        poll();

        /**
         * Polling function for the 18 packets from the LiDar.
         */
        void 
        poll_full();

        /**
         * Getter for the network parameters of the LiDar device. 
         */
        std::pair<std::string, unsigned> 
        get_network_params(); 
    };
}

#endif