/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file dephan_node.cpp
 * @brief ROS node for mechanical LiDar data
 */

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>

#include "ros_driver.hpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    // init ROS
    ros::init(argc, argv);

    // init ros handle node
    ros::NodeHandle nh;

    // load configuration from the json
    if (argc < 2) {
        std::cerr << "path to config file does not provided" << std::endl;
        return 1;
    }
    json configuration = json::parse(std::ifstream{argv[1]});

    // log starting info and configuration
    std::cout << std::endl
              << "Starting driver with following configuration: " << std::endl
              << std::endl;

    for (auto& [k, v] : configuration.items())
        std::cout << k << " : " << v << std::endl;
    std::cout << std::endl;

    // is driver in PCAP mode?
    if (configuration["mode"] == "PCAP")
        dephan_ros::Driver driver(
            nh, configuration.value("pcap_path", "/root/test.pcap"),
            configuration.value("topic", "point_cloud2_data")
        );

    // is driver in UDP mode?
    else if (configuration["mode"] == "UDP")
        dephan_ros::Driver driver(
            nh, configuration.value("ip", "0.0.0.0"),
            configuration.value("port", 3000),
            configuration.value("topic", "point_cloud2_data")
        );

    // error reporting otherwise
    else {
        std::cerr << "bad config" << std::endl;
        return 1;
    }

    // polling device via driver
    while (ros::ok()) {
        driver.poll();
        ros::spinOnce();
    }

    return 0;
}