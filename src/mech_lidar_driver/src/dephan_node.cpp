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
    ros::init(argc, argv, "ss_lidar");

    // init ros handle node
    ros::NodeHandle nh; 

    // init driver
    dephan_ros::Driver driver(nh, "192.168.0.120", 51551, "point_cloud2_data");  
    ROS_INFO("Started"); 

    // polling device via driver
    while(ros::ok()) {
        driver.poll();
        ros::spinOnce();
    }

    return 0; 
}