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
        unsigned port; 
        std::unique_ptr<receiver_socket> socket; 

        std::string pcap_path;
        std::unique_ptr<Tins::FileSniffer> pcap_sniffer;
        long long _prev_pkt_tmstmp;

        ros::Publisher rawdata_publihser; 
        ros::Publisher pointcloud_publisher; 
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
        Driver(ros::NodeHandle nh, std::string ip_addr, unsigned port, std::string topic_name); 
        Driver(ros::NodeHandle nh, std::string pcap_path, std::string topic_name);

        void 
        poll();

        void 
        poll_full();

        std::pair<std::string, unsigned> 
        get_network_params(); 
    };
}

#endif