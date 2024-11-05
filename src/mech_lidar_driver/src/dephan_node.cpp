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
    rclcpp::init(argc, argv);

    // load configuration from the json
    json configuration = json::parse(std::ifstream{argv[1]});

    // log starting info
    std::cout << std::endl
              << "Starting driver with following configuration: " << std::endl
              << std::endl;

    // log configuration details
    for (auto& [k, v] : configuration.items())
        std::cout << k << " : " << v << std::endl;
    std::cout << std::endl;

    // is driver in PCAP mode?
    if (configuration["mode"] == "PCAP")
        rclcpp::spin(std::make_shared<dephan_ros::Driver>(
            configuration.value("pcap_path", "/root/test.pcap"),
            configuration.value("topic", "point_cloud2_data")
        ));

    // us driver in UDP mode?
    else if (configuration["mode"] == "UDP")
        rclcpp::spin(std::make_shared<dephan_ros::Driver>(
            configuration.value("ip", "0.0.0.0"),
            configuration.value("port", 3000),
            configuration.value("topic", "point_cloud2_data")
        ));

    // error reporting otherwise
    else {
        std::cerr << "bad config" << std::endl;
        return 1;
    }

    // stop ros session
    rclcpp::shutdown();

    return 0;
}
