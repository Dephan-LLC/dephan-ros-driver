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
    ros::init(argc, argv, "ss_lidar");
    ros::NodeHandle nh; 

    dephan_ros::Driver driver(nh, "192.168.0.120", 51551, "point_cloud2_data");  
    ROS_INFO("Started"); 

    while(ros::ok()) {
        driver.poll();
        ros::spinOnce();
    }

    return 0; 
}