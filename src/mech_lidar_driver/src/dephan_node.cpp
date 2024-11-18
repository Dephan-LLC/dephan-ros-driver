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

void log_help() {
    std::cout << std::endl
              << "-c / --config" << "\t:\t"
              << "relative path from execute directory to configuration file"
              << std::endl
              << std::endl;
}

json get_configuration(int argc, char* argv[]) {

    // is cmd arguments actually provided?
    if (argc < 2) {
        std::cout << "Config does not provided" << std::endl;
        std::cout << "Use default config otherwise" << std::endl;

        return json::parse(std::ifstream{
            "./src/mech_lidar_driver/configs/default_udp_config.json"
        });
    }

    // if provided it should be a -c or --config
    else {
        if (!std::strcmp(argv[1], "-c") || !std::strcmp(argv[1], "--config"))
            return json::parse(std::ifstream{argv[2]});
        else
            throw std::runtime_error("Bad command line flags");
    }
}

int main(int argc, char* argv[]) {

    // help-request processing
    if (argc == 2 &&
        (!std::strcmp(argv[1], "-h") || !std::strcmp(argv[1], "--help"))) {

        // log help info
        log_help();

        return 0;
    }

    // init configuration
    json configuration = get_configuration(argc, argv);

    // init ROS
    ros::init(argc, argv, "lidar_driver");

    // init ros handle node
    ros::NodeHandle nh;

    // log starting info
    std::cout << std::endl
              << "=================================================="
              << std::endl
              << "Starting driver with the following configuration: "
              << std::endl
              << "=================================================="
              << std::endl;

    // log configuration details
    for (auto& [k, v] : configuration.items())
        std::cout << k << " : " << v << std::endl;
    std::cout << std::endl;

    // is driver in PCAP mode?
    if (configuration["mode"] == "PCAP") {

        // initialize driver instance
        dephan_ros::Driver driver(
            nh, configuration.value("pcap_path", "/root/test.pcap"),
            configuration.value("topic", "point_cloud2_data")
        );

        // polling via driver
        while (ros::ok()) {
            driver.poll();
            ros::spinOnce();
        }
    }

    // is driver in UDP mode?
    else if (configuration["mode"] == "UDP") {

        // initialize driver instance
        dephan_ros::Driver driver(
            nh, configuration.value("ip", "0.0.0.0"),
            configuration.value("port", 3000),
            configuration.value("topic", "point_cloud2_data")
        );

        // polling via driver
        while (ros::ok()) {
            driver.poll();
            ros::spinOnce();
        }
    }

    // error reporting otherwise
    else
        throw std::runtime_error("Unknown configuration mode");

    return 0;
}