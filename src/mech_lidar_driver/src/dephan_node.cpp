/** 
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file dephan_node.cpp
 * @brief ROS node for mechanical LiDar data
 */

#include "ros_driver.hpp"


int main(int argc, char *argv[]) { 
    // init ROS
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<dephan_ros::Driver>("192.168.0.120", 51551, "point_cloud2_data"));

    rclcpp::spin(std::make_shared<dephan_ros::Driver>("/root/test.pcap", "point_cloud2_data_pcap"));

    rclcpp::shutdown();

//////

    // dephan_ros::Driver driver("192.168.0.101", 51551, "point_cloud2_data");  
    // dephan_ros::Driver driver_pcap(nh, "/root/test.pcap", "point_cloud2_data_pcap");

    // driver.poll();
    // driver_pcap.poll();

    // // // polling device via driver
    // while(rclcpp::ok()) {
    //     // driver.poll_full();
    //     // driver_pcap.poll();
    //     driver.poll();
    //     rclcpp::spin(nh);
    // }


    return 0; 
}