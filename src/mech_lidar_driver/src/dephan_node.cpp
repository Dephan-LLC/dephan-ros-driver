/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file dephan_node.cpp
 * @brief Process entry point for the ROS 2 mechanical LiDAR driver.
 */

#include "driver_cli.hpp"
#include "driver_config.hpp"
#include "driver_runtime.hpp"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

int main(int argc, char* argv[]) {
    try {
        std::vector<std::string> arguments(argv, argv + argc);
        const dephan_ros::CliOptions options = dephan_ros::parse_cli(arguments);

        if (options.help) {
            dephan_ros::print_cli_help(
                std::cout, "ros2 run mech_lidar_driver mech_driver"
            );
            return 0;
        }
        if (dephan_ros::has_http_command(options)) {
            return dephan_ros::run_http_command(options, std::cout);
        }

        const auto configuration_json =
            dephan_ros::load_driver_configuration(options, std::cout);
        auto configuration =
            dephan_ros::parse_driver_config(configuration_json);
        return dephan_ros::run_driver_runtime(
            argc, argv, std::move(configuration)
        );
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}
