/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file dephan_node.cpp
 * @brief ROS node for mechanical LiDar data
 */

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "http_client.hpp"
#include "ros_driver.hpp"

using json = nlohmann::json;

namespace {
const char* DEFAULT_DRIVER_CONFIG =
    "./src/mech_lidar_driver/configs/default_udp_config.json";
const char* DEFAULT_HTTP_HOST = "192.168.0.120";

enum class HttpCommandType {
    None,
    Get,
    Post,
    SetConfig,
    Download,
    Stream,
};

struct CliOptions {
    std::string config_path;
    bool config_provided = false;
    bool help            = false;

    std::string http_host = DEFAULT_HTTP_HOST;
    int http_port         = 80;
    int http_timeout_ms   = 3000;

    HttpCommandType http_command = HttpCommandType::None;
    std::string http_path;
    std::string http_body;
    std::string output_path;
    std::string config_name;
    std::string config_value;
};

bool is_arg(const char* value, const char* short_name, const char* long_name) {
    return std::strcmp(value, short_name) == 0 ||
           std::strcmp(value, long_name) == 0;
}

bool is_ros_arg(const std::string& value) {
    return value.find(":=") != std::string::npos ||
           value.find("__") == 0;
}

int parse_int_arg(const std::string& name, const std::string& value) {
    size_t parsed = 0;
    int result    = std::stoi(value, &parsed);
    if (parsed != value.size()) {
        throw std::runtime_error("Invalid integer for " + name + ": " + value);
    }
    return result;
}

void ensure_no_http_command(const CliOptions& options) {
    if (options.http_command != HttpCommandType::None) {
        throw std::runtime_error("Only one HTTP command can be used at a time");
    }
}

void require_value(int argc, int index, const std::string& option) {
    if (index + 1 >= argc) {
        throw std::runtime_error("Missing value for " + option);
    }
}

bool has_next_value(int argc, int index, char* argv[]) {
    return index + 1 < argc && std::string(argv[index + 1]).find("-") != 0;
}

std::string read_file_argument(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open file: " + path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string read_body_argument(const std::string& value) {
    if (!value.empty() && value[0] == '@') {
        if (value.size() == 1) {
            throw std::runtime_error("Missing file path after @");
        }
        return read_file_argument(value.substr(1));
    }

    return value;
}

std::string zone_name_body(const std::string& key, const std::string& name) {
    return json{{key, name}}.dump();
}

void write_binary_file(const std::string& path, const std::string& data) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Failed to open output file: " + path);
    }
    output.write(data.data(), data.size());
    if (!output) {
        throw std::runtime_error("Failed to write output file: " + path);
    }
}

CliOptions parse_cli(int argc, char* argv[]) {
    CliOptions options;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (is_ros_arg(arg)) {
            continue;
        }
        if (is_arg(argv[i], "-h", "--help")) {
            options.help = true;
        }
        else if (is_arg(argv[i], "-c", "--config")) {
            require_value(argc, i, arg);
            options.config_path     = argv[++i];
            options.config_provided = true;
        }
        else if (arg == "--http-host" || arg == "--lidar-ip") {
            require_value(argc, i, arg);
            options.http_host = argv[++i];
        }
        else if (arg == "--http-port") {
            require_value(argc, i, arg);
            options.http_port = parse_int_arg(arg, argv[++i]);
        }
        else if (arg == "--http-timeout-ms") {
            require_value(argc, i, arg);
            options.http_timeout_ms = parse_int_arg(arg, argv[++i]);
        }
        else if (arg == "--get-config") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/config.json";
        }
        else if (arg == "--get-status") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/status.json";
        }
        else if (arg == "--get-features") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/features.json";
        }
        else if (arg == "--get-version") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/version.txt";
        }
        else if (arg == "--get-version-string") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/version_string.txt";
        }
        else if (arg == "--get-log") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/log.txt";
        }
        else if (arg == "--get-timestamp") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/timestamp.txt";
        }
        else if (arg == "--log-events") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Stream;
            options.http_path    = "/log_events";
        }
        else if (arg == "--get-zones") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/zones.json";
        }
        else if (arg == "--set-zones") {
            ensure_no_http_command(options);
            require_value(argc, i, arg);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/zones.json";
            options.http_body    = read_body_argument(argv[++i]);
        }
        else if (arg == "--add-zone") {
            ensure_no_http_command(options);
            require_value(argc, i, arg);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/zones/add.json";
            options.http_body    = read_body_argument(argv[++i]);
        }
        else if (arg == "--delete-zone") {
            ensure_no_http_command(options);
            require_value(argc, i, arg);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/zones/delete.json";
            options.http_body    = zone_name_body("name", argv[++i]);
        }
        else if (arg == "--get-zone-settings") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/zones/settings.json";
        }
        else if (arg == "--set-zone-settings") {
            ensure_no_http_command(options);
            require_value(argc, i, arg);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/zones/settings.json";
            options.http_body    = read_body_argument(argv[++i]);
        }
        else if (arg == "--get-zone-status") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Get;
            options.http_path    = "/zones/status.json";
        }
        else if (arg == "--reset-zones") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/zones/reset";
            if (has_next_value(argc, i, argv)) {
                options.http_body = zone_name_body("zone_name", argv[++i]);
            }
        }
        else if (arg == "--save-zones") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/zones/save";
        }
        else if (arg == "--get-zones-lut") {
            ensure_no_http_command(options);
            require_value(argc, i, arg);
            options.http_command = HttpCommandType::Download;
            options.http_path    = "/zones/lut.bin";
            options.output_path  = argv[++i];
        }
        else if (arg == "--safety-events") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Stream;
            options.http_path    = "/safety_events";
        }
        else if (arg == "--set-config") {
            ensure_no_http_command(options);
            if (i + 2 >= argc) {
                throw std::runtime_error(
                    "Missing values for --set-config <name> <json-token>"
                );
            }
            options.http_command = HttpCommandType::SetConfig;
            options.config_name  = argv[++i];
            options.config_value = argv[++i];
            if (options.config_name.empty() ||
                options.config_name.find('/') != std::string::npos) {
                throw std::runtime_error("Invalid config parameter name");
            }
        }
        else if (arg == "--set-config-json") {
            ensure_no_http_command(options);
            require_value(argc, i, arg);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/config.json";
            options.http_body    = read_body_argument(argv[++i]);
        }
        else if (arg == "--save-preset") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/save_preset.cgi";
        }
        else if (arg == "--jump-to-bootloader") {
            ensure_no_http_command(options);
            options.http_command = HttpCommandType::Post;
            options.http_path    = "/jump_to_bootloader.cgi";
        }
        else {
            throw std::runtime_error("Bad command line flag: " + arg);
        }
    }

    return options;
}

void log_help() {
    std::cout
        << std::endl
        << "Driver mode:" << std::endl
        << "  -c, --config <path>          Path to UDP/PCAP JSON config"
        << std::endl
        << std::endl
        << "HTTP options:" << std::endl
        << "  --http-host <ip-or-host>     Lidar HTTP host, default "
        << DEFAULT_HTTP_HOST << std::endl
        << "  --lidar-ip <ip>              Alias for --http-host" << std::endl
        << "  --http-port <port>           Lidar HTTP port, default 80"
        << std::endl
        << "  --http-timeout-ms <ms>       HTTP timeout, default 3000"
        << std::endl
        << std::endl
        << "HTTP read commands:" << std::endl
        << "  --get-config                 GET /config.json" << std::endl
        << "  --get-status                 GET /status.json" << std::endl
        << "  --get-features               GET /features.json" << std::endl
        << "  --get-version                GET /version.txt" << std::endl
        << "  --get-version-string         GET /version_string.txt"
        << std::endl
        << "  --get-log                    GET /log.txt" << std::endl
        << "  --get-timestamp              GET /timestamp.txt" << std::endl
        << "  --log-events                 GET /log_events" << std::endl
        << std::endl
        << "Safety zones commands:" << std::endl
        << "  --get-zones                  GET /zones.json" << std::endl
        << "  --set-zones <json|@file>     POST /zones.json" << std::endl
        << "  --add-zone <json|@file>      POST /zones/add.json"
        << std::endl
        << "  --delete-zone <name>         POST /zones/delete.json"
        << std::endl
        << "  --get-zone-settings          GET /zones/settings.json"
        << std::endl
        << "  --set-zone-settings <json|@file>"
        << "  POST /zones/settings.json" << std::endl
        << "  --get-zone-status            GET /zones/status.json"
        << std::endl
        << "  --reset-zones [name]         POST /zones/reset" << std::endl
        << "  --save-zones                 POST /zones/save" << std::endl
        << "  --get-zones-lut <path>       GET /zones/lut.bin"
        << std::endl
        << "  --safety-events              GET /safety_events"
        << std::endl
        << std::endl
        << "HTTP write commands:" << std::endl
        << "  --set-config <name> <json-token>"
        << "  POST /config/{name}" << std::endl
        << "  --set-config-json <json|@file>"
        << "  POST /config.json" << std::endl
        << "  --save-preset                POST /save_preset.cgi"
        << std::endl
        << "  --jump-to-bootloader         POST /jump_to_bootloader.cgi"
        << std::endl
        << std::endl
        << "Examples:" << std::endl
        << "  ros2 run mech_lidar_driver mech_driver --get-config"
        << std::endl
        << "  ros2 run mech_lidar_driver mech_driver --lidar-ip 192.168.0.120"
        << " --set-config motor_speed 10" << std::endl
        << "  ros2 run mech_lidar_driver mech_driver --set-config preemptive_conns"
        << " true" << std::endl
        << "  ros2 run mech_lidar_driver mech_driver --set-config-json"
        << " '{\"motor_speed\":10}'" << std::endl
        << "  ros2 run mech_lidar_driver mech_driver --add-zone @zone.json"
        << std::endl
        << "  ros2 run mech_lidar_driver mech_driver --get-zones-lut /tmp/zones_lut.bin"
        << std::endl
        << std::endl;
}

json get_configuration(const CliOptions& options) {
    if (!options.config_provided) {
        std::cout << "Config does not provided" << std::endl;
        std::cout << "Use default config otherwise" << std::endl;

        return json::parse(std::ifstream{DEFAULT_DRIVER_CONFIG});
    }

    return json::parse(std::ifstream{options.config_path});
}

int run_http_command(const CliOptions& options) {
    dephan_ros::HttpClient client(
        options.http_host, options.http_port, options.http_timeout_ms
    );

    dephan_ros::HttpResponse response;
    switch (options.http_command) {
    case HttpCommandType::Get:
        response = client.get(options.http_path);
        break;
    case HttpCommandType::Post:
        response = client.post(options.http_path, options.http_body);
        break;
    case HttpCommandType::SetConfig:
        response = client.post(
            "/config/" + options.config_name, options.config_value
        );
        break;
    case HttpCommandType::Download:
        response = client.get(options.http_path);
        break;
    case HttpCommandType::Stream:
        response = client.stream_get(options.http_path, std::cout, &std::cout);
        return response.status_code >= 200 && response.status_code < 300 ? 0 : 1;
        break;
    case HttpCommandType::None:
        throw std::runtime_error("No HTTP command selected");
    }

    if (options.http_command == HttpCommandType::Download) {
        std::cout << "HTTP " << response.status_code;
        if (!response.reason.empty()) {
            std::cout << " " << response.reason;
        }
        std::cout << std::endl;

        if (response.status_code >= 200 && response.status_code < 300) {
            write_binary_file(options.output_path, response.body);
            std::cout << "Saved " << response.body.size() << " bytes to "
                      << options.output_path << std::endl;
        }
        else if (!response.body.empty()) {
            std::cout << response.body;
            if (response.body.back() != '\n') {
                std::cout << std::endl;
            }
        }

        return response.status_code >= 200 && response.status_code < 300 ? 0 : 1;
    }

    switch (options.http_command) {
    case HttpCommandType::Get:
    case HttpCommandType::Post:
    case HttpCommandType::SetConfig:
    case HttpCommandType::Download:
    case HttpCommandType::Stream:
        break;
    case HttpCommandType::None:
        throw std::runtime_error("No HTTP command selected");
    }

    std::cout << "HTTP " << response.status_code;
    if (!response.reason.empty()) {
        std::cout << " " << response.reason;
    }
    std::cout << std::endl;

    if (!response.body.empty()) {
        std::cout << response.body;
        if (response.body.back() != '\n') {
            std::cout << std::endl;
        }
    }

    return response.status_code >= 200 && response.status_code < 300 ? 0 : 1;
}
} // namespace

int main(int argc, char* argv[]) {
    try {
        CliOptions options = parse_cli(argc, argv);

        if (options.help) {
            log_help();
            return 0;
        }

        if (options.http_command != HttpCommandType::None) {
            return run_http_command(options);
        }

        // init configuration
        json configuration = get_configuration(options);

        // init ROS
        rclcpp::init(argc, argv);

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

            // is driver capture type FULL?
            if (configuration["capture_type"] == "FULL") {
                auto driver = std::make_shared<dephan_ros::Driver>(
                    configuration.value("pcap_path", "/root/test.pcap"),
                    configuration.value("topic", "point_cloud2_pcap"), true,
                    configuration.value("pointcloud_topic", "")
                );

                // polling via driver
                rclcpp::spin(driver);
            }

            // is driver capture type SINGLE?
            else if (configuration["capture_type"] == "SINGLE") {
                auto driver = std::make_shared<dephan_ros::Driver>(
                    configuration.value("pcap_path", "/root/test.pcap"),
                    configuration.value("topic", "point_cloud2_pcap"), false,
                    configuration.value("pointcloud_topic", "")
                );

                // polling via driver
                rclcpp::spin(driver);
            }

            // error reporting otherwise
            else
                throw std::runtime_error(
                    "Unknown configuration \"capture type\""
                );
        }

        // is driver in UDP mode?
        else if (configuration["mode"] == "UDP") {

            // initialize driver instance
            // is driver capture type FULL?
            if (configuration["capture_type"] == "FULL") {
                auto driver = std::make_shared<dephan_ros::Driver>(
                    configuration.value("ip", "0.0.0.0"),
                    configuration.value("port", 3000),
                    configuration.value("topic", "point_cloud2_udp"), true,
                    configuration.value("pointcloud_topic", "")
                );

                // polling via driver
                rclcpp::spin(driver);
            }

            // is driver capture type SINGLE?
            else if (configuration["capture_type"] == "SINGLE") {
                auto driver = std::make_shared<dephan_ros::Driver>(
                    configuration.value("ip", "0.0.0.0"),
                    configuration.value("port", 3000),
                    configuration.value("topic", "point_cloud2_udp"), false,
                    configuration.value("pointcloud_topic", "")
                );

                // polling via driver
                rclcpp::spin(driver);
            }

            // error reporting otherwise
            else
                throw std::runtime_error(
                    "Unknown configuration \"capture type\""
                );
        }

        // error reporting otherwise
        else
            throw std::runtime_error("Unknown configuration \"mode\"");

        rclcpp::shutdown();
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
