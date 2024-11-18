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
#include "colored_cout.h"

using json = nlohmann::json;

void log_help() {
    std::cout
        << std::endl
        << utils::make_colored(
               "-c / --config", utils::Color::white, utils::Style::bold
           )
        << "\t:\t"
        << utils::make_colored(
               "relative path from execute directory to configuration file",
               utils::Color::white
           )
        << std::endl
        << std::endl;
}

json get_configuration(int argc, char* argv[]) {

    // is cmd arguments actually provided?
    if (argc < 2) {
        std::cout << utils::make_colored(
                         "Config does not provided", utils::Color::yellow,
                         utils::Style::bold
                     )
                  << std::endl;
        std::cout << utils::make_colored(
                         "Use default config otherwise", utils::Color::yellow,
                         utils::Style::bold
                     )
                  << std::endl;

        return json::parse(std::ifstream{
            "./src/mech_lidar_driver/configs/default_udp_config.json"
        });
    }

    // if provided it should be a -c or --config
    else {
        if (!std::strcmp(argv[1], "-c") || !std::strcmp(argv[1], "--config")) {
            try {
                return json::parse(std::ifstream{argv[2]});
            }
            catch (const std::exception& ex) {
                throw std::runtime_error(utils::make_colored(
                    "Trobles with opening a json file", utils::Color::red,
                    utils::Style::bold
                ));
            }
        }

        else
            throw std::runtime_error(utils::make_colored(
                "Bad command line flags", utils::Color::red, utils::Style::bold
            ));
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
    rclcpp::init(argc, argv);

    // log starting info
    std::cout << std::endl
              << utils::make_colored(
                     "==================================================",
                     utils::Color::green, utils::Style::bold
                 )
              << std::endl
              << utils::make_colored(
                     "Starting driver with the following configuration: ",
                     utils::Color::green, utils::Style::bold
                 )
              << std::endl
              << utils::make_colored(
                     "==================================================",
                     utils::Color::green, utils::Style::bold
                 )
              << std::endl;

    // log configuration details
    for (auto& [k, v] : configuration.items())
        std::cout << k << " : " << v << std::endl;
    std::cout << std::endl;

    // is the driver in PCAP mode?
    if (configuration["mode"] == "PCAP")
        rclcpp::spin(std::make_shared<dephan_ros::Driver>(
            configuration.value("pcap_path", "/root/test.pcap"),
            configuration.value("topic", "point_cloud2_pcap")
        ));

    // is the driver in UDP mode?
    else if (configuration["mode"] == "UDP")
        rclcpp::spin(std::make_shared<dephan_ros::Driver>(
            configuration.value("ip", "0.0.0.0"),
            configuration.value("port", 3000),
            configuration.value("topic", "point_cloud2_udp")
        ));

    // error reporting otherwise
    else {
        // stop ros session
        rclcpp::shutdown();

        throw std::runtime_error(utils::make_colored(
            "Unknown configuration mode", utils::Color::red, utils::Style::bold
        ));
    }

    // stop ros session
    rclcpp::shutdown();

    return 0;
}
