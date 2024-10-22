#ifndef DRIVER_HPP
#define DRIVER_HPP

#include <ros/ros.h>
#include <memory>
#include <string>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt8MultiArray.h>
#include <pcl_ros/point_cloud.h>

#include "packet_raw.hpp"
#include "reciever_socket.hpp"
#include "packet_handler_mech.hpp"


namespace dephan_ros { 
    class Driver {
    private:
        std::string ip_addr; 
        unsigned port; 
        std::unique_ptr<receiver_socket> socket; 

        ros::Publisher rawdata_publihser; 
        ros::Publisher pointcloud_publisher; 
        ros::Publisher pointcloud2_publisher; 
        
    public:
        Driver(ros::NodeHandle nh, std::string ip_addr, unsigned port, std::string topic_name); 

        void 
        poll();

        void 
        poll_full();

        std::pair<std::string, unsigned> 
        get_network_params(); 
    };
}

#endif